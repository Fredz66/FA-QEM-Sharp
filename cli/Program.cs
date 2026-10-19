using System.Globalization;
using System.Text;
using FaQemSharp;

Console.OutputEncoding = Encoding.UTF8;
return Run(args);

static int Run(string[] args)
{
    const string usage = "Usage : faqem original.glb resultat.glb --triangles 20000 [--overwrite]\nÉcrit aussi resultat.glb.report.txt. L’original est conservé.\nCodes de sortie : 0 = cible atteinte ; 2 = arguments invalides ; 3 = résultat sauvegardé, cible non atteinte ; 1 = échec.";
    if (args.Length == 1 && args[0] is "--help" or "-h") { Console.WriteLine(usage); return 0; }
    if (args.Length is < 4 or > 5 || args[2] != "--triangles" ||
        !int.TryParse(args[3], NumberStyles.None, CultureInfo.InvariantCulture, out int target) || target < 1 ||
        (args.Length == 5 && args[4] != "--overwrite"))
    { Console.Error.WriteLine(usage); return 2; }
    string? modelTemp = null, reportTemp = null;
    try
    {
        string input = Path.GetFullPath(args[0]), output = Path.GetFullPath(args[1]);
        string reportPath = output + ".report.txt";
        if (!OperatingSystem.IsWindows()) throw new PlatformNotSupportedException("Cet outil inclut le runtime OpenCV Windows x64. Pour une autre plateforme, utilisez la bibliothèque avec un runtime OpenCV compatible.");
        if (string.Equals(input, output, StringComparison.OrdinalIgnoreCase)) throw new ArgumentException("Le fichier de sortie doit être différent de l’original.");
        if (!string.Equals(Path.GetExtension(output), ".glb", StringComparison.OrdinalIgnoreCase)) throw new ArgumentException("La sortie doit porter l’extension .glb.");
        bool overwrite = args.Length == 5;
        if (!overwrite && (File.Exists(output) || File.Exists(reportPath))) throw new IOException("La sortie ou son rapport existe déjà. Choisissez un autre nom ou utilisez --overwrite.");
        if (!Directory.Exists(Path.GetDirectoryName(output))) throw new DirectoryNotFoundException("Le dossier de sortie n’existe pas.");
        var result = FaQemPipeline.Process(File.ReadAllBytes(input), target, new ConsoleProgress());
        string suffix = "." + Guid.NewGuid().ToString("N") + ".tmp";
        modelTemp = output + suffix; reportTemp = reportPath + suffix;
        File.WriteAllBytes(modelTemp, result.Glb);
        File.WriteAllText(reportTemp, result.Report.Text);
        File.Move(modelTemp, output, overwrite); modelTemp = null;
        File.Move(reportTemp, reportPath, overwrite); reportTemp = null;
        Console.WriteLine(result.Report.Summary);
        Console.WriteLine($"GLB : {output}\nRapport : {reportPath}");
        if (!result.TargetReached) Console.Error.WriteLine($"Cible non atteinte : {result.Triangles:N0}/{target:N0} triangles. Le résultat est conservé.");
        return result.TargetReached ? 0 : 3;
    }
    catch (Exception e)
    {
        Console.Error.WriteLine("Échec : " + e.Message);
        return 1;
    }
    finally
    {
        foreach (string? path in new[] { modelTemp, reportTemp })
            if (path is not null) try { File.Delete(path); } catch (IOException) { } catch (UnauthorizedAccessException) { }
    }
}

sealed class ConsoleProgress : IProgress<WorkProgress>
{
    readonly object gate = new();
    long last;
    public void Report(WorkProgress value)
    {
        lock (gate)
        {
            long now = Environment.TickCount64;
            if (now - last < 1000) return;
            last = now;
            Console.Error.WriteLine(value.Percent is int p ? $"{p}% — {value.Stage}" : value.Stage);
        }
    }
}
