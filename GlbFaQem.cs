using System.Numerics;
using System.Runtime.InteropServices;
using System.Text.Json.Nodes;
using OpenCvSharp;
using P = FaQemSharp.FaQem.Point;

namespace FaQemSharp;

// FA-QEM's two stages deliberately share no mutable UV/texture data. All render
// corners and images below are immutable source data; only welded positions enter
// the simplifier. Connected UV islands and source-texture reuse are built after
// reduction. Atlas pixels query the reverse-collapse history, never a global
// nearest-surface search. See SurfaceMap for the choices left unspecified by §3.2.3.
internal static partial class GlbFaQem
{
    internal sealed class SourcePrimitive
    {
        public required JsonObject Json;
        public required float[] Positions;
        public float[]? Normals,Tangents,Colors;
        public int ColorSize;
        public Dictionary<int,float[]> Uvs=[];
        public required uint[] Indices;
        public int Material;
        public P Position(int i)=>new(Positions[i*3],Positions[i*3+1],Positions[i*3+2]);
        public P Normal(int i)=>Normals is null?default:new(Normals[i*3],Normals[i*3+1],Normals[i*3+2]);
        public Vector2 Uv(int set,int i) {
            if(!Uvs.TryGetValue(set,out var data))throw new InvalidDataException($"FA-QEM: texture uses missing TEXCOORD_{set}.");
            return new(data[2*i],data[2*i+1]);
        }
    }
    internal readonly record struct SourceFace(SourcePrimitive Primitive,int A,int B,int C)
    {
        public P Normal=>(Primitive.Position(B)-Primitive.Position(A)).Cross(Primitive.Position(C)-Primitive.Position(A)).Unit;
    }
    sealed class MeshSource
    {
        public required JsonObject Json;
        public List<SourceFace> Faces=[];
        public List<P> Points=[];
        public List<uint> Indices=[];
        public List<List<int>> Incident=[];
    }
    sealed class Weld
    {
        const double Tolerance=1e-6;
        readonly Dictionary<(double,double,double),List<int>> cells=[];
        readonly MeshSource mesh;
        public Weld(MeshSource mesh)=>this.mesh=mesh;
        public int Add(P p) {
            var key=(Math.Floor(p.X/Tolerance),Math.Floor(p.Y/Tolerance),Math.Floor(p.Z/Tolerance));
            int best=-1;double distance=Tolerance*Tolerance;
            for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++)
                if(cells.TryGetValue((key.Item1+x,key.Item2+y,key.Item3+z),out var bucket))foreach(int id in bucket) {
                    var delta=mesh.Points[id]-p;double d=delta.Dot(delta);
                    if(d<=distance&&(d<distance||best<0||id<best)) {distance=d;best=id;}
                }
            if(best>=0)return best;
            int result=mesh.Points.Count;mesh.Points.Add(p);mesh.Incident.Add([]);
            if(!cells.TryGetValue(key,out var list))cells[key]=list=[];
            list.Add(result);return result;
        }
    }

    public static MeshReductionResult Reduce(byte[] input,int target,IProgress<WorkProgress>? progress)
    {
        if(target<1)throw new ArgumentOutOfRangeException(nameof(target));
        var doc=new GlbDocument(input);
        if(doc.Root["extensionsUsed"] is JsonArray used&&used.Any(e=>e?.GetValue<string>()=="KHR_animation_pointer"))
            throw new InvalidDataException("FA-QEM does not support animation pointers into rebaked materials.");
        progress?.Report(new("FA-QEM: reading original geometry and appearance",0));
        var meshes=new List<MeshSource>();
        foreach(var meshNode in doc.Root["meshes"]!.AsArray()) {
            var mesh=new MeshSource{Json=meshNode!.AsObject()};var weld=new Weld(mesh);
            foreach(var primitiveNode in mesh.Json["primitives"]!.AsArray()) {
                var json=primitiveNode!.AsObject();
                if((json["mode"]?.GetValue<int>()??4)!=4||json["targets"] is not null||json["extensions"] is not null)
                    throw new InvalidDataException("FA-QEM requires decoded static triangle lists without primitive extensions or morph targets.");
                var attrs=json["attributes"]!.AsObject();
                var positions=doc.ReadFloats(attrs["POSITION"]!.GetValue<int>(),out int dim);
                if(dim!=3||positions.Length==0)throw new InvalidDataException("FA-QEM: invalid positions.");
                int count=positions.Length/3;
                var indices=json["indices"] is JsonNode ix?doc.ReadIndices(ix.GetValue<int>()):Enumerable.Range(0,count).Select(x=>(uint)x).ToArray();
                if(indices.Length==0||indices.Length%3!=0||indices.Any(i=>i>=count))throw new InvalidDataException("FA-QEM: invalid triangle indices.");
                var primitive=new SourcePrimitive{Json=json,Positions=positions,Indices=indices,Material=json["material"]?.GetValue<int>()??-1};
                foreach(var attr in attrs) {
                    if(attr.Key=="POSITION")continue;
                    var values=doc.ReadFloats(attr.Value!.GetValue<int>(),out int components);
                    if(values.Length!=count*components)throw new InvalidDataException("FA-QEM: inconsistent attribute counts.");
                    if(attr.Key=="NORMAL"&&components==3)primitive.Normals=values;
                    else if(attr.Key=="TANGENT"&&components==4)primitive.Tangents=values;
                    else if(attr.Key=="COLOR_0"&&components is 3 or 4) {primitive.Colors=values;primitive.ColorSize=components;}
                    else if(attr.Key.StartsWith("TEXCOORD_")&&components==2&&int.TryParse(attr.Key[9..],out int set))primitive.Uvs.Add(set,values);
                    else throw new InvalidDataException($"FA-QEM: attribute {attr.Key} cannot be transferred by this static PBR pipeline.");
                }
                var remap=new int[count];Array.Fill(remap,-1);
                foreach(uint index in indices)if(remap[index]<0)remap[index]=weld.Add(primitive.Position((int)index));
                for(int i=0;i<indices.Length;i+=3) {
                    int face=mesh.Faces.Count,a=(int)indices[i],b=(int)indices[i+1],c=(int)indices[i+2];
                    mesh.Faces.Add(new(primitive,a,b,c));
                    foreach(int vertex in new[]{remap[a],remap[b],remap[c]}) {mesh.Indices.Add((uint)vertex);mesh.Incident[vertex].Add(face);}
                }
            }
            if(mesh.Faces.Count==0)throw new InvalidDataException("FA-QEM: empty mesh.");
            meshes.Add(mesh);
        }
        // Scene nodes and transforms remain intact. Geometry shared by instances
        // is reduced once. All primitives of each mesh are reduced TOGETHER.
        if(doc.Root["nodes"] is JsonArray nodes&&nodes.Any(n=>n?["skin"] is not null))
            throw new InvalidDataException("FA-QEM currently requires a static mesh, without skinning.");
        using var textures=new TextureStore(doc);
        foreach(var primitive in meshes.SelectMany(m=>m.Faces.Select(f=>f.Primitive)).Distinct())textures.Validate(primitive);
        long inputCount=meshes.Sum(m=>(long)m.Faces.Count),remaining=Math.Min(target,inputCount),capacity=inputCount;
        long total=0;var reports=new List<string>();int pages=0;
        for(int meshIndex=0;meshIndex<meshes.Count;meshIndex++) {
            var mesh=meshes[meshIndex];
            int allocation=(int)Math.Max(1,remaining*mesh.Faces.Count/Math.Max(1,capacity));
            allocation=Math.Min(allocation,mesh.Faces.Count);
            var positions=mesh.Points.SelectMany(p=>new[]{(float)p.X,(float)p.Y,(float)p.Z}).ToArray();
            var result=FaQem.Simplify(positions,mesh.Indices.ToArray(),allocation,progress);
            reports.Add($"Maillage {meshIndex+1}: {result.Diagnostics}");
            remaining-=result.Indices.Length/3;remaining=Math.Max(0,remaining);capacity-=mesh.Faces.Count;
            if(result.Indices.Length==0)throw new InvalidDataException("FA-QEM removed every face of a mesh. The original model has been retained.");
            total+=result.Indices.Length/3;
            var map=new SurfaceMap(mesh,result,positions);
            var reuse=FindReusableUvs(map,textures,progress);
            var outputPrimitives=new JsonArray();
            foreach(var group in Enumerable.Range(0,result.SourceFaces.Length).GroupBy(i=>mesh.Faces[result.SourceFaces[i]].Primitive.Material)) {
                foreach(var primitive in BakeUvIslands(doc,textures,map,group.ToArray(),group.Key,reuse,progress,reports)) {
                    outputPrimitives.Add(primitive);pages++;
                }
            }
            mesh.Json["primitives"]=outputPrimitives;
            if(map.ProvenanceFallbacks>0)reports.Add($"Transfert : {map.ProvenanceFallbacks:N0} échantillons (marges comprises) ont repris la face source d’origine, faute de face compatible dans le voisinage du parent retrouvé.");
        }
        CompactAppearance(doc.Root);
        // Drop superseded encodings, then declare the actual atlas format below.
        foreach(string property in new[]{"extensionsUsed","extensionsRequired"})if(doc.Root[property] is JsonArray extensions)
            for(int i=extensions.Count-1;i>=0;i--)if(extensions[i]!.GetValue<string>() is "KHR_mesh_quantization" or "KHR_texture_basisu")extensions.RemoveAt(i);
        bool hasWebp=doc.Root["textures"]!.AsArray().Any(t=>t?["extensions"]?["EXT_texture_webp"] is not null);
        foreach(string property in new[]{"extensionsUsed","extensionsRequired"}) {
            if(doc.Root[property] is not JsonArray list)doc.Root[property]=list=new JsonArray();
            for(int i=list.Count-1;i>=0;i--)if(list[i]!.GetValue<string>()=="EXT_texture_webp")list.RemoveAt(i);
            if(hasWebp)list.Add("EXT_texture_webp");
        }
        progress?.Report(new("FA-QEM: writing textured GLB",100));
        string diagnostics=string.Join(Environment.NewLine,reports)+$"\n{pages} primitives PBR ; transfert par historique inverse des contractions. Aucun verrou UV. Îlots connexes projetés sans recouvrement, angle maximal 15°, densité au moins égale à la référence par triangle ; nouveaux échantillons de cuisson. Images source réutilisées sans réencodage lorsque la correspondance UV passe les contrôles ; cuisson de secours sinon. Normales recuites dans la base de tangentes du résultat. PNG/WebP sans perte pour les nouvelles images.";
        var output=doc.Serialize();
        diagnostics+=$"\nPoids GLB : {input.Length/1e6:F3} → {output.Length/1e6:F3} Mo ({100.0*(output.Length/(double)input.Length-1):+0.0;-0.0;0} %).";
        if(output.Length>input.Length)diagnostics+=" Le résultat reste plus lourd que l’original : aucune baisse de résolution ni compression avec perte n’a été appliquée pour forcer ce seuil.";
        return new(output,total,0,0,diagnostics);
    }

    static void CompactAppearance(JsonObject root)
    {
        // Newly baked primitives reference newly created materials only. Retaining
        // the old material/image arrays would keep the entire original textures
        // alive in Document.CompactBuffers and double exported file size.
        void Remap(string arrayName,IEnumerable<(JsonObject Owner,string Key)> references) {
            var original=root[arrayName]!.AsArray();var replacement=new JsonArray();var mapping=new Dictionary<int,int>();
            foreach(var (owner,key) in references) {
                int old=owner[key]!.GetValue<int>();
                if(!mapping.TryGetValue(old,out int id)) {id=replacement.Count;mapping.Add(old,id);replacement.Add(original[old]!.DeepClone());}
                owner[key]=id;
            }
            root[arrayName]=replacement;
        }
        Remap("materials",root["meshes"]!.AsArray().SelectMany(m=>m!["primitives"]!.AsArray()).Select(p=>(p!.AsObject(),"material")));
        var infos=new List<(JsonObject,string)>();
        foreach(var material in root["materials"]!.AsArray()) {
            foreach(string key in new[]{"normalTexture","occlusionTexture","emissiveTexture"})if(material![key] is JsonObject info)infos.Add((info,"index"));
            foreach(string key in new[]{"baseColorTexture","metallicRoughnessTexture"})if(material!["pbrMetallicRoughness"]?[key] is JsonObject info)infos.Add((info,"index"));
        }
        Remap("textures",infos);
        // Reused textures can have an optional sampler or both a core source
        // and extension source. Preserve/remap every retained image reference.
        var imageRefs=new List<(JsonObject,string)>();
        foreach(var node in root["textures"]!.AsArray()) {
            var texture=node!.AsObject();
            // Match TextureStore's source selection; discard unused compressed fallbacks.
            if(texture["source"] is not null) {
                imageRefs.Add((texture,"source"));
                if(texture["extensions"] is JsonObject ext){ext.Remove("EXT_texture_webp");ext.Remove("KHR_texture_basisu");}
            } else if(texture["extensions"]?["EXT_texture_webp"] is JsonObject webp) {
                imageRefs.Add((webp,"source"));texture["extensions"]!.AsObject().Remove("KHR_texture_basisu");
            }
        }
        Remap("images",imageRefs);
        if(root["samplers"] is not null)Remap("samplers",root["textures"]!.AsArray().Where(t=>t?["sampler"] is not null).Select(t=>(t!.AsObject(),"sampler")));
    }

    readonly record struct Sample(int Face,P Barycentric);

    sealed class SurfaceMap
    {
        public readonly MeshSource Source;
        public readonly FaQem.Result Reduction;
        public readonly P[] Positions,Normals,GeometricNormals;
        public readonly int[][] Corners;
        public readonly P[][] CornerNormals;
        long provenanceFallbacks;
        public long ProvenanceFallbacks=>Interlocked.Read(ref provenanceFallbacks);
        public SurfaceMap(MeshSource source,FaQem.Result reduction,float[] positions) {
            Source=source;Reduction=reduction;
            Positions=Enumerable.Range(0,positions.Length/3).Select(i=>new P(positions[i*3],positions[i*3+1],positions[i*3+2])).ToArray();
            Normals=new P[Positions.Length];
            Corners=new int[reduction.SourceFaces.Length][];CornerNormals=new P[Corners.Length][];GeometricNormals=new P[Corners.Length];
            for(int i=0;i<reduction.Indices.Length;i+=3) {
                int a=(int)reduction.Indices[i],b=(int)reduction.Indices[i+1],c=(int)reduction.Indices[i+2];
                P n=(Positions[b]-Positions[a]).Cross(Positions[c]-Positions[a]);Normals[a]+=n;Normals[b]+=n;Normals[c]+=n;
                Corners[i/3]=[a,b,c];GeometricNormals[i/3]=n.Unit;
            }
            for(int i=0;i<Normals.Length;i++)Normals[i]=Normals[i].Unit;
            for(int i=0;i<Corners.Length;i++) {
                var n=Corners[i].Select(v=>Normals[v]).ToArray();
                // Opposite incident surfaces in non-manifold input can cancel
                // vertex normals. Use a valid per-face frame; the normal texture
                // still transfers the original shading normals into this frame.
                if(n.Any(x=>x.Dot(GeometricNormals[i])<=1e-3))n=[GeometricNormals[i],GeometricNormals[i],GeometricNormals[i]];
                CornerNormals[i]=n;
            }
        }
        // §3.2.3 specifies nearest-parent reassignment, but not the initial sample
        // association, source face selection at UV seams, nor interpolation inside
        // that face. Explicit completion of these unspecified details:
        // 1. Associate the texel with the closest corner of its FINAL triangle.
        // 2. Undo its contractions, selecting the closer pre-collapse parent each time.
        // 3. Start in that original vertex's one-ring, then descend the distance
        //    over connected source faces on the same material/side. Stopping at
        //    the first one-ring clips samples to unrelated patch boundaries when
        //    neighboring texels take different branches of the collapse history.
        //    This walk interpolates ON the original connected surface; it does
        //    not blend UVs from different charts or search disconnected surfaces.
        //    No global projection, ray cast, UV interpolation through a collapse,
        //    or nearest-neighbor substitution for the history is performed.
        // The retained original face is a deterministic fallback at material seams.
        // This is reproducible, but cannot establish bitwise equivalence to the
        // unpublished author implementation. Source points remain fixed during undo.
        public Sample Query(int finalFace,P p,P faceNormal) {
            int start=finalFace*3,vertex=(int)Reduction.Indices[start];double best=double.PositiveInfinity;
            for(int k=0;k<3;k++) {int id=(int)Reduction.Indices[start+k];var d=Positions[id]-p;double ds=d.Dot(d);if(ds<best){best=ds;vertex=id;}}
            int leaf=Reduction.MapToSource(vertex,p);
            int seed=Reduction.SourceFaces[finalFace],material=Source.Faces[seed].Primitive.Material;
            int chosen=-1;P bary=default;best=double.PositiveInfinity;
            void Consider(int face) {
                var f=Source.Faces[face];
                if(f.Primitive.Material!=material||f.Normal.Dot(faceNormal)<=0)return;
                var b=ClosestBarycentric(p,f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C));
                P q=Blend(f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C),b),d=q-p;
                double ds=d.Dot(d);
                if(ds<best){best=ds;chosen=face;bary=b;}
            }
            foreach(int face in Source.Incident[leaf])Consider(face);
            if(chosen<0) {
                Interlocked.Increment(ref provenanceFallbacks);
                chosen=seed;var f=Source.Faces[seed];
                bary=ClosestBarycentric(p,f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C));
                var d=Blend(f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C),bary)-p;best=d.Dot(d);
            }
            // Strictly decreasing distance guarantees termination without an
            // arbitrary hop limit that would reintroduce visible patch borders.
            // Source adjacency uses welded geometry, so UV seams remain traversable.
            while(true) {
                int previous=chosen;
                for(int corner=0;corner<3;corner++)
                    foreach(int neighbor in Source.Incident[(int)Source.Indices[previous*3+corner]])Consider(neighbor);
                if(chosen==previous)break;
            }
            return new(chosen,bary);
        }
    }

    static P Blend(P a,P b,P c,P w)=>a*w.X+b*w.Y+c*w.Z;
    static Vector2 Blend(Vector2 a,Vector2 b,Vector2 c,P w)=>a*(float)w.X+b*(float)w.Y+c*(float)w.Z;
    // Voronoi-region closest point, with a segment fallback for degenerate source faces.
    internal static P ClosestBarycentric(P p,P a,P b,P c) {
        P ab=b-a,ac=c-a,ap=p-a;double d1=ab.Dot(ap),d2=ac.Dot(ap);
        if(d1<=0&&d2<=0)return new(1,0,0);
        P bp=p-b;double d3=ab.Dot(bp),d4=ac.Dot(bp);
        if(d3>=0&&d4<=d3)return new(0,1,0);
        double vc=d1*d4-d3*d2;
        if(vc<=0&&d1>=0&&d3<=0&&d1!=d3){double v=d1/(d1-d3);return new(1-v,v,0);}
        P cp=p-c;double d5=ab.Dot(cp),d6=ac.Dot(cp);
        if(d6>=0&&d5<=d6)return new(0,0,1);
        double vb=d5*d2-d1*d6;
        if(vb<=0&&d2>=0&&d6<=0&&d2!=d6){double w=d2/(d2-d6);return new(1-w,0,w);}
        double va=d3*d6-d5*d4;
        if(va<=0&&d4-d3>=0&&d5-d6>=0&&d4-d3+d5-d6>0){double w=(d4-d3)/(d4-d3+d5-d6);return new(0,1-w,w);}
        double sum=va+vb+vc;
        if(sum>0&&double.IsFinite(sum))return new(va/sum,vb/sum,vc/sum);
        // Collinear triangles occur only in the original, never in the atlas.
        double best=double.PositiveInfinity;P result=new(1,0,0);
        foreach(var (x,y,wx,wy) in new[]{(a,b,new P(1,0,0),new P(0,1,0)),(b,c,new P(0,1,0),new P(0,0,1)),(c,a,new P(0,0,1),new P(1,0,0))}) {
            P e=y-x;double t=e.Dot(e)>0?Math.Clamp((p-x).Dot(e)/e.Dot(e),0,1):0;P d=x+e*t-p;
            if(d.Dot(d)<best){best=d.Dot(d);result=wx*(1-t)+wy*t;}
        }
        return result;
    }

    internal sealed class ImageData
    {
        public readonly int Width,Height;
        public readonly byte[] Bgra;
        public ImageData(byte[] encoded) {
            using var decoded=Cv2.ImDecode(encoded,ImreadModes.Unchanged);
            if(decoded.Empty()||decoded.Depth()!=MatType.CV_8U)throw new InvalidDataException("FA-QEM: unsupported or invalid 8-bit source texture.");
            Width=decoded.Width;Height=decoded.Height;
            using var rgba=new Mat();
            if(decoded.Channels()==4)decoded.CopyTo(rgba);
            else if(decoded.Channels()==3)Cv2.CvtColor(decoded,rgba,ColorConversionCodes.BGR2BGRA);
            else if(decoded.Channels()==1)Cv2.CvtColor(decoded,rgba,ColorConversionCodes.GRAY2BGRA);
            else throw new InvalidDataException("FA-QEM: unsupported texture channel count.");
            Bgra=new byte[checked(Width*Height*4)];
            for(int y=0;y<Height;y++)Marshal.Copy(rgba.Ptr(y),Bgra,y*Width*4,Width*4);
        }
        public int Size=>Math.Max(Width,Height);
        public Vector4 Sample(Vector2 uv,int wrapS,int wrapT,bool srgb,bool nearest) {
            double u=Wrap(uv.X,wrapS),v=Wrap(uv.Y,wrapT);
            double x=u*Width-0.5,y=v*Height-0.5;
            int ix=(int)Math.Floor(x),iy=(int)Math.Floor(y);float fx=(float)(x-ix),fy=(float)(y-iy);
            Vector4 Pixel(int px,int py) {
                px=Index(px,Width,wrapS);py=Index(py,Height,wrapT);int offset=(py*Width+px)*4;
                var value=new Vector4(Bgra[offset+2]/255f,Bgra[offset+1]/255f,Bgra[offset]/255f,Bgra[offset+3]/255f);
                if(srgb){value.X=Linear(value.X);value.Y=Linear(value.Y);value.Z=Linear(value.Z);}return value;
            }
            if(nearest)return Pixel((int)Math.Floor(u*Width),(int)Math.Floor(v*Height));
            return Vector4.Lerp(Vector4.Lerp(Pixel(ix,iy),Pixel(ix+1,iy),fx),Vector4.Lerp(Pixel(ix,iy+1),Pixel(ix+1,iy+1),fx),fy);
        }
        static double Wrap(double x,int mode) {
            if(mode==33071)return Math.Clamp(x,0,1);
            if(mode==33648){x-=Math.Floor(x/2)*2;return x<=1?x:2-x;}
            return x-Math.Floor(x);
        }
        static int Index(int x,int size,int mode) {
            if(mode==33071)return Math.Clamp(x,0,size-1);
            if(mode==33648){x=((x%(size*2))+size*2)%(size*2);return x<size?x:size*2-1-x;}
            return ((x%size)+size)%size;
        }
    }
    internal sealed class TextureBinding(ImageData image,int uvSet,Vector2 offset,Vector2 scale,float rotation,int wrapS,int wrapT,bool nearest)
    {
        public readonly ImageData Image=image;
        public readonly int UvSet=uvSet;
        public Vector2 Transform(Vector2 uv) {
            uv*=scale;float c=MathF.Cos(rotation),s=MathF.Sin(rotation);
            return offset+new Vector2(c*uv.X-s*uv.Y,s*uv.X+c*uv.Y);
        }
        public Vector4 SampleUv(Vector2 uv,bool srgb)=>Image.Sample(Transform(uv),wrapS,wrapT,srgb,nearest);
        public Vector4 Sample(SourceFace f,P bary,bool srgb) {
            var p=f.Primitive;var uv=Blend(p.Uv(UvSet,f.A),p.Uv(UvSet,f.B),p.Uv(UvSet,f.C),bary);
            return Image.Sample(Transform(uv),wrapS,wrapT,srgb,nearest);
        }
    }
    internal sealed class MaterialData
    {
        public required JsonObject Json;
        public TextureBinding? Base,MetalRough,Normal,Occlusion,Emissive;
        public float NormalScale=1;
        public int Size=>new[]{Base,MetalRough,Normal,Occlusion,Emissive}.Where(x=>x is not null).Select(x=>x!.Image.Size).DefaultIfEmpty(1024).Max();
    }
    internal sealed class TextureStore(GlbDocument doc):IDisposable
    {
        readonly Dictionary<int,ImageData> images=[];
        readonly Dictionary<int,MaterialData> materials=[];
        public void Dispose(){images.Clear();materials.Clear();}
        public MaterialData Material(int id) {
            if(materials.TryGetValue(id,out var found))return found;
            JsonObject json=id<0?new JsonObject():doc.Root["materials"]![id]!.AsObject();
            if(json["extensions"] is JsonObject extensions)foreach(var e in extensions)
                if(e.Key is not ("KHR_materials_unlit" or "KHR_materials_emissive_strength"))
                    throw new InvalidDataException($"FA-QEM: material extension {e.Key} has no implemented atlas transfer; the original model is retained.");
            var pbr=json["pbrMetallicRoughness"];
            var value=new MaterialData{Json=json,Base=Binding(pbr?["baseColorTexture"]),MetalRough=Binding(pbr?["metallicRoughnessTexture"]),
                Normal=Binding(json["normalTexture"]),Occlusion=Binding(json["occlusionTexture"]),Emissive=Binding(json["emissiveTexture"]),
                NormalScale=json["normalTexture"]?["scale"]?.GetValue<float>()??1};
            materials.Add(id,value);return value;
        }
        public void Validate(SourcePrimitive primitive) {
            var m=Material(primitive.Material);
            foreach(var b in new[]{m.Base,m.MetalRough,m.Normal,m.Occlusion,m.Emissive})if(b is not null&&!primitive.Uvs.ContainsKey(b.UvSet))
                throw new InvalidDataException($"FA-QEM: missing TEXCOORD_{b.UvSet} required by a source texture.");
        }
        internal TextureBinding? Binding(JsonNode? info) {
            if(info is null)return null;
            var texture=doc.Root["textures"]![info["index"]!.GetValue<int>()]!;
            int source=texture["source"]?.GetValue<int>()??texture["extensions"]?["EXT_texture_webp"]?["source"]?.GetValue<int>()??
                throw new InvalidDataException("FA-QEM requires a decoded PNG/JPEG/WebP texture source (KTX2-only textures are unsupported).");
            if(!images.TryGetValue(source,out var image)) {
                var node=doc.Root["images"]![source]!;byte[] bytes;
                if(node["bufferView"] is JsonNode view)bytes=doc.ReadEmbeddedView(view.GetValue<int>());
                else if(node["uri"]?.GetValue<string>() is string uri&&uri.StartsWith("data:")&&uri.Contains(";base64,"))bytes=Convert.FromBase64String(uri[(uri.IndexOf(',')+1)..]);
                else throw new InvalidDataException("FA-QEM requires embedded texture images.");
                image=new ImageData(bytes);images.Add(source,image);
            }
            var transform=info["extensions"]?["KHR_texture_transform"];
            Vector2 ReadVector(string key,Vector2 fallback)=>transform?[key] is JsonArray a?new(a[0]!.GetValue<float>(),a[1]!.GetValue<float>()):fallback;
            int set=transform?["texCoord"]?.GetValue<int>()??info["texCoord"]?.GetValue<int>()??0;
            var sampler=texture["sampler"] is JsonNode sid?doc.Root["samplers"]![sid.GetValue<int>()]:null;
            return new(image,set,ReadVector("offset",Vector2.Zero),ReadVector("scale",Vector2.One),transform?["rotation"]?.GetValue<float>()??0,
                sampler?["wrapS"]?.GetValue<int>()??10497,sampler?["wrapT"]?.GetValue<int>()??10497,sampler?["magFilter"]?.GetValue<int>()==9728);
        }
    }

    static float Linear(float s)=>s<=0.04045f?s/12.92f:MathF.Pow((s+0.055f)/1.055f,2.4f);
    static float Srgb(float s)=>s<=0.0031308f?12.92f*s:1.055f*MathF.Pow(s,1/2.4f)-0.055f;
    static byte Byte(float x)=>(byte)Math.Clamp((int)MathF.Round(x*255),0,255);
    static void WritePixel(byte[] pixels,int index,Vector4 color,bool srgb) {
        if(srgb){color.X=Srgb(color.X);color.Y=Srgb(color.Y);color.Z=Srgb(color.Z);}
        pixels[index]=Byte(color.Z);pixels[index+1]=Byte(color.Y);pixels[index+2]=Byte(color.X);pixels[index+3]=Byte(color.W);
    }
    // Choose the smaller lossless encoding; transparent WebP additionally
    // requires byte-exact decoding to protect hidden RGB and edge filtering.
    static (byte[] Bytes,string Mime) Encode(byte[] bgra,int width,int height) {
        using var mat=new Mat(height,width,MatType.CV_8UC4);
        Marshal.Copy(bgra,0,mat.Data,bgra.Length);
        if(!Cv2.ImEncode(".png",mat,out var png,new[]{16,9}))throw new InvalidDataException("PNG encoding failed.");
        bool opaque=true;for(int i=3;i<bgra.Length;i+=4)if(bgra[i]!=255){opaque=false;break;}
        if(opaque) {
            using var rgb=new Mat();Cv2.CvtColor(mat,rgb,ColorConversionCodes.BGRA2BGR);
            // OpenCV IMWRITE_WEBP_QUALITY=64; values >100 request lossless.
            if(Cv2.ImEncode(".webp",rgb,out var webp,new[]{64,101})&&webp.Length<png.Length)return(webp,"image/webp");
        }
        else if(Cv2.ImEncode(".webp",mat,out var webp,new[]{64,101})&&webp.Length<png.Length) {
            // Lossless WebP may discard RGB under alpha=0. Accept transparent
            // WebP only when decoding reproduces every channel, including hidden RGB.
            using var decoded=Cv2.ImDecode(webp,ImreadModes.Unchanged);
            if(decoded.Type()==MatType.CV_8UC4&&decoded.Width==width&&decoded.Height==height) {
                var row=new byte[width*4];bool exact=true;
                for(int y=0;y<height&&exact;y++) {
                    Marshal.Copy(decoded.Ptr(y),row,0,row.Length);
                    exact=row.AsSpan().SequenceEqual(bgra.AsSpan(y*row.Length,row.Length));
                }
                if(exact)return(webp,"image/webp");
            }
        }
        return(png,"image/png");
    }

    static P Tangent(P edge1,P edge2,Vector2 uv1,Vector2 uv2,P normal,out double sign) {
        double determinant=uv1.X*uv2.Y-uv1.Y*uv2.X;
        P t,b;
        if(Math.Abs(determinant)>1e-20) {t=(edge1*uv2.Y-edge2*uv1.Y)*(1/determinant);b=(edge2*uv1.X-edge1*uv2.X)*(1/determinant);}
        else {t=edge1;b=normal.Cross(t);}
        t=(t-normal*normal.Dot(t)).Unit;
        if(t.Length==0)t=normal.Cross(Math.Abs(normal.X)<0.9?new P(1,0,0):new P(0,1,0)).Unit;
        sign=normal.Cross(t).Dot(b)<0?-1:1;return t;
    }
    internal static P SourceNormal(SourceFace face,P bary,MaterialData material) {
        var src=face.Primitive;
        P n=src.Normals is null?face.Normal:Blend(src.Normal(face.A),src.Normal(face.B),src.Normal(face.C),bary).Unit;
        if(n.Length==0)n=face.Normal;
        if(material.Normal is not TextureBinding binding)return n;
        P tangent;double sign;
        if(src.Tangents is float[] values) {
            P Read(int id)=>new(values[4*id],values[4*id+1],values[4*id+2]);
            tangent=Blend(Read(face.A),Read(face.B),Read(face.C),bary);tangent=(tangent-n*n.Dot(tangent)).Unit;
            sign=values[4*face.A+3]*bary.X+values[4*face.B+3]*bary.Y+values[4*face.C+3]*bary.Z<0?-1:1;
        } else {
            Vector2 a=binding.Transform(src.Uv(binding.UvSet,face.A)),b=binding.Transform(src.Uv(binding.UvSet,face.B)),c=binding.Transform(src.Uv(binding.UvSet,face.C));
            tangent=Tangent(src.Position(face.B)-src.Position(face.A),src.Position(face.C)-src.Position(face.A),b-a,c-a,n,out sign);
        }
        var texel=binding.Sample(face,bary,false);
        P mapped=new((texel.X*2-1)*material.NormalScale,(texel.Y*2-1)*material.NormalScale,texel.Z*2-1);
        return (tangent*mapped.X+n.Cross(tangent)*(sign*mapped.Y)+n*mapped.Z).Unit;
    }

}
