namespace FaQemSharp;

public sealed record WorkProgress(string Stage, int? Percent = null);
public sealed record MeshReductionResult(byte[] Glb, long Triangles, int ProtectedVertices, float Error, string Diagnostics);
public sealed record FaQemOutput(byte[] Glb, long Triangles, bool TargetReached, MeshQualityReport Report);

/// <summary>CPU FA-QEM with the fixed, high-fidelity atlas settings.</summary>
public static class FaQemPipeline
{
    /// <summary>Reduce and bake a self-contained GLB. The caller owns file I/O and scheduling.</summary>
    public static MeshReductionResult Reduce(byte[] original, int targetTriangles, IProgress<WorkProgress>? progress = null)
    {
        ArgumentNullException.ThrowIfNull(original);
        ArgumentOutOfRangeException.ThrowIfLessThan(targetTriangles, 1);
        return GlbFaQem.Reduce(original, targetTriangles, progress);
    }

    /// <summary>Reduce, bake and compare against the supplied original using the quality report.</summary>
    public static FaQemOutput Process(byte[] original, int targetTriangles, IProgress<WorkProgress>? progress = null)
    {
        var result = Reduce(original, targetTriangles, progress);
        bool reached = result.Triangles <= targetTriangles;
        var report = MeshQualityReport.Create(original, result.Glb,
            $"Méthode : fa-qem. Cible : {targetTriangles:N0} triangles ; résultat : {result.Triangles:N0}." +
            (reached ? "" : " Cible non atteinte ; résultat conservé.") + "\r\n" + result.Diagnostics, progress);
        return new(result.Glb, result.Triangles, reached, report);
    }
}
