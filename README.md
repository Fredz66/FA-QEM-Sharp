# FA-QEM Sharp

Independent C# implementation of FA-QEM for textured GLB simplification.

The CPU pipeline combines geometric edge contractions, reverse-collapse surface mapping, connected UV islands, conditional reuse of original images, lossless atlas encoding and sampled quality reports. This repository contains the library and a command-line tool.

## Requirements

- .NET 10.
- OpenCvSharp4 4.13.0.20260627 (managed dependency supplied by NuGet).
- A compatible native OpenCV runtime supplied by the host application. On Windows x64, use `OpenCvSharp4.runtime.win` version `4.13.0.20260627`.

There is no dependency on a GUI, a generation server, CUDA or meshoptimizer. Other operating systems have not been validated. Native PNG and WebP codecs are required; identical encoded bytes across OpenCV versions or platforms are not guaranteed.

## Usage

Reference `FaQemSharp.csproj` from your C# project and add the native runtime appropriate to your platform.

```csharp
using FaQemSharp;

byte[] original = File.ReadAllBytes("original.glb");
FaQemOutput result = FaQemPipeline.Process(original, targetTriangles: 20000);
File.WriteAllBytes("reduced.glb", result.Glb);
File.WriteAllText("reduced.glb.report.txt", result.Report.Text);
Console.WriteLine(result.Report.Summary);
Console.WriteLine($"Target reached: {result.TargetReached}");
```

`Process` performs reduction, atlas baking and comparison against the supplied original. `Reduce` returns the reduced GLB and diagnostics without computing the report; call `MeshQualityReport.Create` separately if needed. Both accept `IProgress<WorkProgress>`. Calls are synchronous: schedule them on a worker thread in GUI applications. Cooperative cancellation is not currently available. The input byte array is not modified. Keep the original file for subsequent reductions.

`GlbDocument` exposes the GLB reader/writer used by the pipeline for host integrations. Report text and most diagnostics are currently in French. This is an early API; its shape may evolve before version 1.0.

## Command-line tool

`FaQemSharp.Cli` is a .NET tool for Windows x64 that bundles the native OpenCV runtime.

```powershell
dotnet tool install --global FaQemSharp.Cli
faqem original.glb reduced.glb --triangles 20000 [--overwrite]
```

It writes `reduced.glb` and `reduced.glb.report.txt`, prints the report summary and leaves the original untouched. Existing outputs are kept unless `--overwrite` is given. Exit codes: `0` target reached, `3` result saved but target not reached, `2` invalid arguments, `1` failure. Messages are in French.

## Build and package

```powershell
dotnet build FaQemSharp.csproj -c Release
dotnet pack FaQemSharp.csproj -c Release -o artifacts/packages
dotnet pack cli/FaQemSharp.Cli.csproj -c Release -o artifacts/packages
```

This produces `FaQemSharp.<version>.nupkg` and `FaQemSharp.Cli.<version>.nupkg`. Add that directory as a local NuGet source alongside your normal source for dependencies, then reference the package and the native runtime in your application. NuGet publication of both packages is triggered manually through the `Publish NuGet` GitHub Actions workflow (`publish-nuget.yml`) on `main`; versions already on NuGet.org are skipped. It uses NuGet Trusted Publishing for the NuGet account `Fredz`, scoped to this repository, without storing a permanent API key. A matching NuGet policy must be configured before the first run. Each published package version is immutable; update the project version before publishing another release.

## Method and scope

Based on [the FA-QEM paper and supplement, arXiv:2605.14029v1](https://arxiv.org/html/2605.14029v1). See also [the authors' project](https://b22237.github.io/FA-QEM/).

This is an independent implementation, not an official author release. Publication ambiguities and implementation decisions are recorded in `FaQem.cs` and `GlbFaQem.cs`, including boundary ordering and the reverse-collapse mapping. Optional virtual edges between disconnected components are not implemented. The paper's performance and quality results have not been reproduced by this project.

The library uses fixed settings: connected charts with at most 64 triangles and 15-degree normal deviation, reference texture density, four-sample baking, and lossless PNG/WebP encoding. Texture reuse is accepted only after local fidelity and encoded-size checks. Lossless encoding does not imply lossless appearance: simplification and resampling can introduce differences.

Input must be a self-contained GLB 2.0 with embedded images and buffer, decoded static triangle primitives and supported attributes. Compressed geometry, sparse geometry accessors, morph targets, skinning attributes and unsupported primitive extensions are rejected. The requested triangle count may not be reached. Local geometry checks do not guarantee manifoldness or absence of self-intersections. Final textures can make the reduced GLB larger than the original.

Reports compare sampled geometry, base color, shading normals and material values against the original. These metrics do not guarantee the absence of local visual defects and are not a complete rendered-image comparison. No test suite or benchmark dataset is included.

## License

MIT — Copyright (c) 2026 Frédéric Lopez. See [LICENSE](LICENSE).

Third-party dependencies retain their own licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The MIT license applies to this implementation, not to the paper, authors' assets or external dependencies.
