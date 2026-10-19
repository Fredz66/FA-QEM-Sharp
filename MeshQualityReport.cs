using System.Globalization;
using System.Numerics;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json.Nodes;
using P = FaQemSharp.FaQem.Point;
using Face = FaQemSharp.GlbFaQem.SourceFace;

namespace FaQemSharp;

// Production report. Measurements are sampled surface comparisons, not a
// renderer benchmark or a certificate of manifoldness / self-intersection freedom.
public sealed class MeshQualityReport
{
    public readonly List<(string Measure,string Original,string Result)> Rows=[];
    public string Notes="";
    public string Summary="";
    public required string Hash;
    public string Text {
        get {
            var s=new StringBuilder("FA-QEM Sharp — compte-rendu du modèle\r\n");
            s.AppendLine(Summary).AppendLine().AppendLine("Détails techniques");
            s.AppendLine($"UTC : {Created:O}\r\nSHA-256 du GLB : {Hash}");
            s.AppendLine("Mesure | Original | Résultat");
            foreach(var r in Rows)s.AppendLine($"{r.Measure} | {r.Original} | {r.Result}");
            return s.AppendLine().AppendLine(Notes).ToString();
        }
    }
    readonly DateTime Created=DateTime.UtcNow;
    static string N(double n,string format="N3")=>n.ToString(format,CultureInfo.GetCultureInfo("fr-FR"));
    public static MeshQualityReport Create(byte[]? original,byte[] result,string context,IProgress<WorkProgress>? progress=null)
    {
        var report=new MeshQualityReport{Hash=Convert.ToHexString(SHA256.HashData(result)),Notes=context+(original is null?"":"\r\nSHA-256 original : "+Convert.ToHexString(SHA256.HashData(original)))};
        report.Summary="Poids du GLB : "+N(result.Length/1e6,"N1")+" Mo.";
        if(original is not null&&original.Length>0) {
            long delta=(long)result.Length-original.Length;
            report.Summary+=" Original : "+N(original.Length/1e6,"N1")+" Mo.\r\n"+
                (delta==0?"Poids identique à l’original.":N(Math.Abs(delta)/1e6,"N2")+" Mo "+(delta>0?"de plus":"de moins")+" que l’original ("+N(100.0*Math.Abs(delta)/original.Length,"N1")+" %)." );
        }
        void Row(string name,string a,string b)=>report.Rows.Add((name,a,b));
        Row("Taille du GLB (Mo)",original is null?"—":N(original.Length/1e6),N(result.Length/1e6));
        try {
            progress?.Report(new("Analyse de la géométrie et des textures"));
            using var dst=new Model(result);using var src=original is null?null:new Model(original);
            report.Summary="Triangles : "+(src is null?"":N(src.Faces.Count,"N0")+" → ")+N(dst.Faces.Count,"N0")+".\r\n"+report.Summary+
                "\r\nTextures : "+N(dst.ImageBytes/1e6,"N1")+" Mo, soit "+N(100.0*dst.ImageBytes/result.Length,"N0")+" % du GLB."+
                "\r\nGéométrie : "+dst.Duplicates+" doublon(s) orienté(s), "+dst.Zero+" triangle(s) d’aire nulle.\r\nComposantes séparées (triangles) : "+dst.Components+".";
            Row("Triangles",src is null?"—":N(src.Faces.Count,"N0"),N(dst.Faces.Count,"N0"));
            Row("Textures",src?.Images??"—",dst.Images);
            Row("Poids des images (Mo)",src is null?"—":N(src.ImageBytes/1e6),N(dst.ImageBytes/1e6));
            Row("Surface (unités locales²)",src is null?"—":N(src.Area),N(dst.Area));
            Row("Triangles d’aire nulle",src?.Zero.ToString()??"—",dst.Zero.ToString());
            Row("Doublons orientés",src?.Duplicates.ToString()??"—",dst.Duplicates.ToString());
            Row("Composantes (triangles)",src?.Components??"—",dst.Components);
            Row("Arêtes incidentes à plus de 2 faces",src?.NonManifoldEdges.ToString()??"—",dst.NonManifoldEdges.ToString());
            if(src is not null&&src.Area>0)Row("Variation de surface","—",N(100*(dst.Area/src.Area-1))+" %");
            bool comparable=src is not null&&src.MeshCount==1&&dst.MeshCount==1&&src.Area>0&&dst.Area>0&&src.Diagonal>0&&
                JsonNode.DeepEquals(src.Doc.Root["nodes"],dst.Doc.Root["nodes"])&&src.Doc.Root["skins"] is null&&src.Doc.Root["animations"] is null&&dst.Doc.Root["skins"] is null&&dst.Doc.Root["animations"] is null;
            if(comparable) {
                progress?.Report(new("Comparaison des surfaces : 20 000 échantillons"));
                var tree=new Tree(dst.Faces);var random=new Random(104729);var distances=new double[20000];
                double colorError=0,roughnessError=0,metallicError=0;string? colorFailure=null,normalFailure=null,materialFailure=null;var normalAngles=new double[distances.Length];
                for(int i=0;i<distances.Length;i++) {
                    double area=random.NextDouble()*src!.Area;int fi=Array.BinarySearch(src.Cumulative,area);if(fi<0)fi=~fi;fi=Math.Min(fi,src.Faces.Count-1);
                    var f=src.Faces[fi];double u=Math.Sqrt(random.NextDouble()),v=random.NextDouble();P w=new(1-u,u*(1-v),u*v);
                    P point=Position(f,w);var hit=tree.Nearest(point);distances[i]=Math.Sqrt(hit.Distance);
                    if(colorFailure is null)try {
                        var delta=src.Color(f,w)-dst.Color(dst.Faces[hit.Face],hit.Bary);
                        colorError+=delta.X*delta.X+delta.Y*delta.Y+delta.Z*delta.Z;
                    } catch(Exception e) when(e is not OutOfMemoryException) {colorFailure=e.Message;}
                    if(normalFailure is null)try {
                        P a=src.Normal(f,w),b=dst.Normal(dst.Faces[hit.Face],hit.Bary);
                        if(a.Length==0||b.Length==0)throw new InvalidDataException("Normale nulle.");
                        normalAngles[i]=Math.Acos(Math.Clamp(a.Dot(b),-1,1))*180/Math.PI;
                    } catch(Exception e) when(e is not OutOfMemoryException) {normalFailure=e.Message;}
                    if(materialFailure is null)try {
                        var delta=Vector2.Abs(src.MetalRough(f,w)-dst.MetalRough(dst.Faces[hit.Face],hit.Bary));
                        roughnessError+=delta.X;metallicError+=delta.Y;
                    } catch(Exception e) when(e is not OutOfMemoryException) {materialFailure=e.Message;}
                }
                Array.Sort(distances);
                Row("Distance moyenne original → résultat / diagonale","—",N(100*distances.Average()/src!.Diagonal)+" %");
                Row("Distance au 95e percentile / diagonale","—",N(100*distances[(int)Math.Ceiling(.95*distances.Length)-1]/src.Diagonal)+" %");
                Row("Fidélité couleur, PSNR (RGB linéaire)","—",colorFailure is not null?"N/D":colorError==0?"∞ dB":N(-10*Math.Log10(colorError/(3*distances.Length)),"N2")+" dB");
                report.Summary+="\r\nÉcart géométrique moyen : "+N(100*distances.Average()/src.Diagonal)+" % de la diagonale de l’original."+
                    "\r\nFidélité couleur : "+(colorFailure is not null?"non mesurée":colorError==0?"identique aux points mesurés":N(-10*Math.Log10(colorError/(3*distances.Length)),"N2")+" dB (plus élevé = plus fidèle)")+"."+
                    "\r\nComparaison échantillonnée avec l’original ; elle ne garantit pas l’absence de défauts visuels locaux.";
                if(colorFailure is not null)report.Notes+="\r\nCouleur non mesurée : "+colorFailure;
                Array.Sort(normalAngles);
                Row("Écart moyen des normales de shading","—",normalFailure is null?N(normalAngles.Average(),"N2")+"°":"N/D");
                Row("Écart des normales au 95e percentile","—",normalFailure is null?N(normalAngles[(int)Math.Ceiling(.95*normalAngles.Length)-1],"N2")+"°":"N/D");
                if(normalFailure is not null)report.Notes+="\r\nNormales non mesurées : "+normalFailure;
                Row("Erreur moyenne absolue de rugosité (0–1)","—",materialFailure is null?N(roughnessError/distances.Length,"N5"):"N/D");
                Row("Erreur moyenne absolue de métallicité (0–1)","—",materialFailure is null?N(metallicError/distances.Length,"N5"):"N/D");
                if(materialFailure is not null)report.Notes+="\r\nMatériau non mesuré : "+materialFailure;
            } else {
                report.Summary+="\r\nFidélité : comparaison avec l’original indisponible (voir les détails).";
                Row("Distance moyenne original → résultat","—","N/D");Row("Distance au 95e percentile","—","N/D");Row("Fidélité couleur, PSNR","—","N/D");
                report.Notes+="\r\nComparaison non calculée : aucune référence distincte, géométrie vide, plusieurs maillages, animation ou placement incompatible.";
            }
            report.Notes+="\r\nNormales de shading : normales interpolées et normal maps décodées dans leur base de tangentes, comparées en coordonnées locales aux mêmes points de surface. Ces angles ne constituent pas une comparaison de rendu éclairé.\r\nDistances : 20 000 points déterministes, tirés uniformément en aire sur l’original, vers le triangle le plus proche du résultat ; normalisation par la diagonale de l’original en coordonnées locales. Ce ne sont pas des maxima de Hausdorff.\r\nCouleur : baseColor, facteurs et couleurs de sommets en RGB linéaire, aux points correspondants ; alpha non évalué, ni éclairage, ni normal map, ni évaluation PBR complète. La correspondance peut être ambiguë sur les surfaces proches ou superposées.\r\nComposantes et doublons : positions exactement identiques regroupées par maillage, indépendamment des UV. Les faces de sens opposé ne sont pas comptées comme doublons orientés. Aucune garantie manifold ou d’absence d’auto-intersection. Zéro triangle d’aire nulle est un résultat positif.\r\nLa taille du GLB exclut le compagnon .pxsource et ce rapport. Les dimensions des textures sont celles des images encodées, pas une mesure de fidélité.";
        } catch(Exception e) when(e is not OutOfMemoryException) {report.Summary+="\r\nAnalyse incomplète : voir les détails.";report.Notes+="\r\nAnalyse partielle, mesures indisponibles : "+e.Message;}
        return report;
    }
    static P Position(Face f,P w)=>f.Primitive.Position(f.A)*w.X+f.Primitive.Position(f.B)*w.Y+f.Primitive.Position(f.C)*w.Z;
    sealed class Model:IDisposable
    {
        public readonly GlbDocument Doc;
        readonly GlbFaQem.TextureStore textures;
        readonly Dictionary<int,GlbFaQem.TextureBinding?> bindings=[];
        public readonly List<Face> Faces=[];
        public readonly double[] Cumulative;
        public readonly double Area,Diagonal;
        public readonly int MeshCount,Zero,Duplicates,NonManifoldEdges;
        public readonly string Images,Components;
        public readonly long ImageBytes;
        public Model(byte[] bytes) {
            Doc=new(bytes);textures=new(Doc);
            var dimensions=new List<string>();
            if(Doc.Root["images"] is JsonArray images)foreach(var image in images) {
                byte[] data=image?["bufferView"] is JsonNode view?Doc.ReadEmbeddedView(view.GetValue<int>()):
                    image?["uri"]?.GetValue<string>() is string uri&&uri.StartsWith("data:")?Convert.FromBase64String(uri[(uri.IndexOf(',')+1)..]):throw new InvalidDataException("Image externe : dimensions indisponibles.");
                ImageBytes+=data.Length;
                using var decoded=OpenCvSharp.Cv2.ImDecode(data,OpenCvSharp.ImreadModes.Unchanged);
                if(decoded.Empty())throw new InvalidDataException("Format d’image non décodable.");
                dimensions.Add($"{decoded.Width} × {decoded.Height} ({image?["mimeType"]?.GetValue<string>()??"image"})");
            }
            Images=dimensions.Count==0?"Aucune":string.Join(" ; ",dimensions.GroupBy(x=>x).Select(g=>$"{g.Count()} × {g.Key}"));
            var welded=new Dictionary<(int,P),int>();var parent=new List<int>();var faceVertices=new List<(int,int,int)>();
            var keys=new HashSet<(int,int,int)>();var edges=new Dictionary<(int,int),int>();
            int Root(int x){while(parent[x]!=x){parent[x]=parent[parent[x]];x=parent[x];}return x;}
            int Vertex(int mesh,P p){if(welded.TryGetValue((mesh,p),out int id))return id;id=parent.Count;parent.Add(id);welded.Add((mesh,p),id);return id;}
            void Edge(int a,int b){var k=a<b?(a,b):(b,a);edges.TryGetValue(k,out int n);edges[k]=n+1;}
            P min=new(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity),max=new(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity);
            var cumulative=new List<double>();
            foreach(var mesh in Doc.Root["meshes"]!.AsArray()) {
                int meshId=MeshCount++;
                foreach(var node in mesh!["primitives"]!.AsArray()) {
                    var json=node!.AsObject();if((json["mode"]?.GetValue<int>()??4)!=4)throw new InvalidDataException("Analyse des modes autres que TRIANGLES indisponible.");
                    var attrs=json["attributes"]!.AsObject();var positions=Doc.ReadFloats(attrs["POSITION"]!.GetValue<int>(),out int width);
                    if(width!=3)throw new InvalidDataException("Positions invalides.");
                    var indices=json["indices"] is JsonNode ix?Doc.ReadIndices(ix.GetValue<int>()):Enumerable.Range(0,positions.Length/3).Select(i=>(uint)i).ToArray();
                    var primitive=new GlbFaQem.SourcePrimitive{Json=json,Positions=positions,Indices=indices,Material=json["material"]?.GetValue<int>()??-1};
                    foreach(var attr in attrs) {
                        if(attr.Key.StartsWith("TEXCOORD_")&&int.TryParse(attr.Key[9..],out int set))primitive.Uvs[set]=Doc.ReadFloats(attr.Value!.GetValue<int>(),out _);
                        if(attr.Key=="COLOR_0")primitive.Colors=Doc.ReadFloats(attr.Value!.GetValue<int>(),out primitive.ColorSize);
                        if(attr.Key=="NORMAL")primitive.Normals=Doc.ReadFloats(attr.Value!.GetValue<int>(),out _);
                        if(attr.Key=="TANGENT")primitive.Tangents=Doc.ReadFloats(attr.Value!.GetValue<int>(),out _);
                    }
                    for(int i=0;i<indices.Length;i+=3) {
                        var f=new Face(primitive,(int)indices[i],(int)indices[i+1],(int)indices[i+2]);Faces.Add(f);
                        P a=primitive.Position(f.A),b=primitive.Position(f.B),c=primitive.Position(f.C);
                        double area=(b-a).Cross(c-a).Length*.5;if(area==0)Zero++;Area+=area;cumulative.Add(Area);
                        foreach(P p in new[]{a,b,c}){min=Min(min,p);max=Max(max,p);}
                        int x=Vertex(meshId,a),y=Vertex(meshId,b),z=Vertex(meshId,c);parent[Root(y)]=Root(x);parent[Root(z)]=Root(x);
                        var key=x<=y&&x<=z?(x,y,z):y<=z?(y,z,x):(z,x,y);if(!keys.Add(key))Duplicates++;
                        faceVertices.Add((x,y,z));Edge(x,y);Edge(y,z);Edge(z,x);
                    }
                }
            }
            Cumulative=cumulative.ToArray();Diagonal=(max-min).Length;
            NonManifoldEdges=edges.Count(e=>e.Value>2);
            var counts=faceVertices.GroupBy(f=>Root(f.Item1)).Select(g=>g.Count()).OrderDescending().ToArray();
            Components=$"{counts.Length} ({string.Join(", ",counts.Take(20))}{(counts.Length>20?", …":"")})";
        }
        public Vector4 Color(Face f,P bary) {
            int id=f.Primitive.Material;var pbr=id<0?null:Doc.Root["materials"]?[id]?["pbrMetallicRoughness"];
            if(!bindings.TryGetValue(id,out var binding)){binding=textures.Binding(pbr?["baseColorTexture"]);bindings.Add(id,binding);}
            Vector4 color=binding?.Sample(f,bary,true)??Vector4.One;
            if(pbr?["baseColorFactor"] is JsonArray factor)color*=new Vector4(factor[0]!.GetValue<float>(),factor[1]!.GetValue<float>(),factor[2]!.GetValue<float>(),factor[3]!.GetValue<float>());
            if(f.Primitive.Colors is float[] colors) {
                int w=f.Primitive.ColorSize;
                Vector4 At(int i)=>new(colors[i*w],colors[i*w+1],colors[i*w+2],w==4?colors[i*w+3]:1);
                color*=At(f.A)*(float)bary.X+At(f.B)*(float)bary.Y+At(f.C)*(float)bary.Z;
            }
            return color;
        }
        public Vector2 MetalRough(Face face,P bary) {
            var material=textures.Material(face.Primitive.Material);var pbr=material.Json["pbrMetallicRoughness"];
            var sample=material.MetalRough?.Sample(face,bary,false)??Vector4.One;
            return new(sample.Y*(pbr?["roughnessFactor"]?.GetValue<float>()??1),sample.Z*(pbr?["metallicFactor"]?.GetValue<float>()??1));
        }
        public P Normal(Face face,P bary)=>GlbFaQem.SourceNormal(face,bary,textures.Material(face.Primitive.Material));
        public void Dispose()=>textures.Dispose();
    }
    static P Min(P a,P b)=>new(Math.Min(a.X,b.X),Math.Min(a.Y,b.Y),Math.Min(a.Z,b.Z));
    static P Max(P a,P b)=>new(Math.Max(a.X,b.X),Math.Max(a.Y,b.Y),Math.Max(a.Z,b.Z));
    sealed class Tree
    {
        sealed class Node {public P Min,Max;public int Start,Count;public Node? Left,Right;}
        readonly List<Face> faces;readonly int[] ids;readonly Node root;
        public Tree(List<Face> faces){this.faces=faces;ids=Enumerable.Range(0,faces.Count).ToArray();root=Build(0,ids.Length);}
        Node Build(int start,int count) {
            var n=new Node{Start=start,Count=count,Min=new(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity),Max=new(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity)};
            for(int i=start;i<start+count;i++){var f=faces[ids[i]];foreach(int v in new[]{f.A,f.B,f.C}){P p=f.Primitive.Position(v);n.Min=Min(n.Min,p);n.Max=Max(n.Max,p);}}
            if(count<=12)return n;
            P d=n.Max-n.Min;int axis=d.X>=d.Y&&d.X>=d.Z?0:d.Y>=d.Z?1:2;
            double Center(int id){P p=Position(faces[id],new(1d/3,1d/3,1d/3));return axis==0?p.X:axis==1?p.Y:p.Z;}
            Array.Sort(ids,start,count,Comparer<int>.Create((a,b)=>{int c=Center(a).CompareTo(Center(b));return c==0?a.CompareTo(b):c;}));
            int half=count/2;n.Left=Build(start,half);n.Right=Build(start+half,count-half);return n;
        }
        public (int Face,P Bary,double Distance) Nearest(P p) {
            int best=-1;P bary=default;double distance=double.PositiveInfinity;
            double Bound(Node n){double x=Math.Max(n.Min.X-p.X,Math.Max(0,p.X-n.Max.X)),y=Math.Max(n.Min.Y-p.Y,Math.Max(0,p.Y-n.Max.Y)),z=Math.Max(n.Min.Z-p.Z,Math.Max(0,p.Z-n.Max.Z));return x*x+y*y+z*z;}
            void Visit(Node n) {
                if(Bound(n)>distance)return;
                if(n.Left is not null){var a=n.Left;var b=n.Right!;if(Bound(a)>Bound(b))(a,b)=(b,a);Visit(a);Visit(b);return;}
                for(int i=n.Start;i<n.Start+n.Count;i++){int id=ids[i];var f=faces[id];P w=GlbFaQem.ClosestBarycentric(p,f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C));P delta=Position(f,w)-p;double d=delta.Dot(delta);if(d<distance){distance=d;best=id;bary=w;}}
            }
            Visit(root);return(best,bary,distance);
        }
    }
}
