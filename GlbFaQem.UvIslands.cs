using System.Numerics;
using System.Text.Json.Nodes;
using P = FaQemSharp.FaQem.Point;

namespace FaQemSharp;

internal static partial class GlbFaQem
{
    // This is an appearance stage only: no UV constraint enters the QEM queue.
    sealed class UvReuse
    {
        public readonly Vector2[][] Corners,MetalCorners;
        public readonly int[] Mask;
        public UvReuse(int count){Corners=new Vector2[count][];MetalCorners=new Vector2[count][];Mask=new int[count];}
    }
    readonly record struct UvEdge(int A,int B,Vector2 U,Vector2 V);
    readonly record struct CornerChoice(Vector2 Uv,double Distance,int Face);

    static UvReuse FindReusableUvs(SurfaceMap map,TextureStore store,IProgress<WorkProgress>? progress)
    {
        progress?.Report(new("FA-QEM: reconstructing source UV islands"));
        var src=map.Source;int count=src.Faces.Count;
        var parents=Enumerable.Range(0,count).ToArray();
        int Root(int i){while(parents[i]!=i){parents[i]=parents[parents[i]];i=parents[i];}return i;}
        var edges=new Dictionary<UvEdge,int>();
        for(int i=0;i<count;i++) {
            var f=src.Faces[i];if(!f.Primitive.Uvs.ContainsKey(0))continue;
            var ids=new[]{f.A,f.B,f.C};
            for(int k=0;k<3;k++) {
                int a=(int)src.Indices[i*3+k],b=(int)src.Indices[i*3+(k+1)%3];
                Vector2 u=f.Primitive.Uv(0,ids[k]),v=f.Primitive.Uv(0,ids[(k+1)%3]);
                if(a>b){(a,b)=(b,a);(u,v)=(v,u);}
                var key=new UvEdge(a,b,u,v);
                if(edges.TryGetValue(key,out int other)&&src.Faces[other].Primitive.Material==f.Primitive.Material)parents[Root(i)]=Root(other);
                else edges.TryAdd(key,i);
            }
        }
        var reuse=new UvReuse(map.Reduction.SourceFaces.Length);var charts=new int[reuse.Mask.Length][];
        var choices=new Dictionary<(int Vertex,int Chart),CornerChoice>();
        for(int i=0;i<reuse.Mask.Length;i++) {
            var material=store.Material(src.Faces[map.Reduction.SourceFaces[i]].Primitive.Material);
            if(material.Base?.UvSet!=0&&material.MetalRough?.UvSet!=0)continue;
            var vertices=FaceVertices(map,i);P normal=FaceNormal(map,i);var hits=new Sample[3];bool valid=true;
            for(int k=0;k<3;k++) {
                hits[k]=map.Query(i,map.Positions[vertices[k]],normal);
                if(!src.Faces[hits[k].Face].Primitive.Uvs.ContainsKey(0)){valid=false;break;}
            }
            if(!valid)continue;
            charts[i]=hits.Select(h=>Root(h.Face)).ToArray();
            for(int k=0;k<3;k++) {
                var hit=hits[k];var f=src.Faces[hit.Face];P w=hit.Barycentric;
                Vector2 uv=Blend(f.Primitive.Uv(0,f.A),f.Primitive.Uv(0,f.B),f.Primitive.Uv(0,f.C),w);
                P delta=Blend(f.Primitive.Position(f.A),f.Primitive.Position(f.B),f.Primitive.Position(f.C),w)-map.Positions[vertices[k]];
                var key=(vertices[k],charts[i][k]);var candidate=new CornerChoice(uv,delta.Dot(delta),hit.Face);
                if(!choices.TryGetValue(key,out var old)||candidate.Distance<old.Distance||(candidate.Distance==old.Distance&&candidate.Face<old.Face))choices[key]=candidate;
            }
        }
        for(int i=0;i<reuse.Mask.Length;i++) {
            if(charts[i] is null)continue;
            var material=store.Material(src.Faces[map.Reduction.SourceFaces[i]].Primitive.Material);var ids=FaceVertices(map,i);P normal=FaceNormal(map,i);
            var uv=Enumerable.Range(0,3).Select(k=>choices[(ids[k],charts[i][k])].Uv).ToArray();reuse.Corners[i]=uv;
            var center=map.Query(i,Blend(map.Positions[ids[0]],map.Positions[ids[1]],map.Positions[ids[2]],new(1d/3,1d/3,1d/3)),normal);
            var cf=src.Faces[center.Face];
            if(!cf.Primitive.Uvs.ContainsKey(0))continue;
            Vector2 fixedUv=Blend(cf.Primitive.Uv(0,cf.A),cf.Primitive.Uv(0,cf.B),cf.Primitive.Uv(0,cf.C),center.Barycentric);
            bool color=material.Base?.UvSet==0&&charts[i].All(c=>c==charts[i][0])&&Math.Abs(Cross(uv[1]-uv[0],uv[2]-uv[0]))>1e-14;
            bool metal=material.MetalRough?.UvSet==0,constantMetal=metal;
            // Texture channels are independent. A metallic/roughness patch may
            // be constant even where the color texture has a seam or high detail.
            for(int y=0;y<=8&&(color||metal||constantMetal);y++)for(int x=0;x<=8-y;x++) {
                P w=new(1-(x+y)/8d,x/8d,y/8d);
                var hit=map.Query(i,Blend(map.Positions[ids[0]],map.Positions[ids[1]],map.Positions[ids[2]],w),normal);var source=src.Faces[hit.Face];
                Vector2 sampleUv=Blend(uv[0],uv[1],uv[2],w);
                if(color&&(Root(hit.Face)!=charts[i][0]||source.Primitive.Colors is not null||!MatchesBinding(material.Base,source,hit.Barycentric,sampleUv,true,true)))color=false;
                if(metal&&!MatchesBinding(material.MetalRough,source,hit.Barycentric,sampleUv,false,false))metal=false;
                if(constantMetal&&!MatchesBinding(material.MetalRough,source,hit.Barycentric,fixedUv,false,false))constantMetal=false;
            }
            if(color)reuse.Mask[i]|=1;
            if(metal||constantMetal){reuse.Mask[i]|=2;reuse.MetalCorners[i]=metal?uv:[fixedUv,fixedUv,fixedUv];}
            if(i%256==0)progress?.Report(new("FA-QEM: validating original texture coordinates",100*i/reuse.Mask.Length));
        }
        return reuse;
    }
    static bool MatchesBinding(TextureBinding? binding,SourceFace source,P bary,Vector2 uv,bool srgb,bool checkPosition)
    {
        if(binding is null||!source.Primitive.Uvs.ContainsKey(binding.UvSet))return false;
        if(checkPosition) {
            Vector2 sourceUv=Blend(source.Primitive.Uv(binding.UvSet,source.A),source.Primitive.Uv(binding.UvSet,source.B),source.Primitive.Uv(binding.UvSet,source.C),bary);
            Vector2 a=binding.Transform(sourceUv),b=binding.Transform(uv);
            if(Math.Abs(a.X-b.X)*binding.Image.Width>.125||Math.Abs(a.Y-b.Y)*binding.Image.Height>.125)return false;
        }
        Vector4 delta=Vector4.Abs(binding.Sample(source,bary,srgb)-binding.SampleUv(uv,srgb));
        return Math.Max(Math.Max(delta.X,delta.Y),Math.Max(delta.Z,delta.W))<=1f/1024;
    }
    static int[] FaceVertices(SurfaceMap map,int face)=>map.Corners[face];
    static P FaceNormal(SurfaceMap map,int face)=>map.GeometricNormals[face];
    static double Cross(Vector2 a,Vector2 b)=>(double)a.X*b.Y-(double)a.Y*b.X;

    sealed class UvIsland
    {
        public readonly List<int> Faces=[];
        public required P Origin,U,V,Normal;
        public double Scale,MinX,MinY,MaxX,MaxY,StretchV=1;
        public int Width,Height,X,Y;
        public Vector2 Project(P p)=>new((float)((p-Origin).Dot(U)),(float)((p-Origin).Dot(V)*StretchV));
        public Vector2 Pixel(P p){var q=Project(p);return new((float)(2.5+(q.X-MinX)*Scale),(float)(2.5+(q.Y-MinY)*Scale));}
    }
    static bool InteriorOverlap(Vector2[] a,Vector2[] b)
    {
        // Strict separating-axis test: sharing an edge or vertex is permitted.
        foreach(var polygon in new[]{a,b})for(int k=0;k<3;k++) {
            Vector2 e=polygon[(k+1)%3]-polygon[k],axis=new(-e.Y,e.X);
            double amin=double.PositiveInfinity,amax=double.NegativeInfinity,bmin=amin,bmax=amax;
            foreach(var p in a){double d=Vector2.Dot(p,axis);amin=Math.Min(amin,d);amax=Math.Max(amax,d);}
            foreach(var p in b){double d=Vector2.Dot(p,axis);bmin=Math.Min(bmin,d);bmax=Math.Max(bmax,d);}
            if(Math.Min(amax,bmax)-Math.Max(amin,bmin)<=1e-10*axis.Length())return false;
        }
        return true;
    }
    static List<UvIsland> BuildIslands(SurfaceMap map,int[] faces,int referenceCell)
    {
        var neighbors=faces.ToDictionary(f=>f,_=>new SortedSet<int>());var edgeFaces=new Dictionary<(int,int),List<int>>();
        var densities=new Dictionary<int,double>();var budgets=new Dictionary<int,double>();
        foreach(int f in faces) {
            var ids=FaceVertices(map,f);P a=map.Positions[ids[0]],b=map.Positions[ids[1]],c=map.Positions[ids[2]];
            double longest=Math.Max((b-a).Length,Math.Max((c-b).Length,(a-c).Length));
            densities[f]=(referenceCell-5)/longest;
            double altitude=(b-a).Cross(c-a).Length/longest;
            budgets[f]=referenceCell*Math.Min(referenceCell,Math.Ceiling(5+Math.Max(.5,altitude*densities[f])));
            for(int k=0;k<3;k++){int x=ids[k],y=ids[(k+1)%3];var key=x<y?(x,y):(y,x);if(!edgeFaces.TryGetValue(key,out var list))edgeFaces[key]=list=[];list.Add(f);}
        }
        // Non-manifold edges are chart boundaries, not simplification constraints.
        foreach(var edge in edgeFaces.Values)if(edge.Count==2){neighbors[edge[0]].Add(edge[1]);neighbors[edge[1]].Add(edge[0]);}
        var done=new HashSet<int>();var result=new List<UvIsland>();
        foreach(int seed in faces) {
            if(done.Contains(seed))continue;
            var ids=FaceVertices(map,seed);P a=map.Positions[ids[0]],b=map.Positions[ids[1]],c=map.Positions[ids[2]];
            if((c-b).Length>(b-a).Length&&(c-b).Length>=(a-c).Length)(a,b,c)=(b,c,a);else if((a-c).Length>(b-a).Length)(a,b,c)=(c,a,b);
            P normal=(b-a).Cross(c-a).Unit,u=(b-a).Unit;
            var island=new UvIsland{Origin=a,U=u,V=normal.Cross(u),Normal=normal};
            double altitude=(b-a).Cross(c-a).Length/(b-a).Length;
            island.StretchV=Math.Max(1,.5/(altitude*densities[seed]));
            var projected=new List<Vector2[]>();var chartNormals=new Dictionary<int,P>();double budget=0;
            bool Add(int f,bool first) {
                double cosine=FaceNormal(map,f).Dot(normal);if(cosine<.965925826)return false; // max 15 degrees; retain the measured high-fidelity grouping
                var faceVertices=FaceVertices(map,f);
                if(map.CornerNormals[f].Any(n=>n.Dot(normal)<=1e-3))return false;
                for(int k=0;k<3;k++)if(chartNormals.TryGetValue(faceVertices[k],out var shared)&&shared!=map.CornerNormals[f][k])return false;
                var points=FaceVertices(map,f).Select(v=>island.Project(map.Positions[v])).ToArray();
                if(!first&&projected.Any(t=>InteriorOverlap(t,points)))return false;
                double scale=Math.Max(island.Scale,densities[f]/cosine);
                double minX=first?points.Min(p=>(double)p.X):Math.Min(island.MinX,points.Min(p=>(double)p.X));
                double minY=first?points.Min(p=>(double)p.Y):Math.Min(island.MinY,points.Min(p=>(double)p.Y));
                double maxX=first?points.Max(p=>(double)p.X):Math.Max(island.MaxX,points.Max(p=>(double)p.X));
                double maxY=first?points.Max(p=>(double)p.Y):Math.Max(island.MaxY,points.Max(p=>(double)p.Y));
                int width=(int)Math.Ceiling((maxX-minX)*scale-1e-4)+5,height=(int)Math.Ceiling((maxY-minY)*scale-1e-4)+5;
                if(!first&&(width>4096||height>4096||(double)width*height>budget+budgets[f]))return false;
                island.Scale=scale;island.MinX=minX;island.MinY=minY;island.MaxX=maxX;island.MaxY=maxY;island.Width=width;island.Height=height;
                island.Faces.Add(f);projected.Add(points);budget+=budgets[f];done.Add(f);
                for(int k=0;k<3;k++)chartNormals[faceVertices[k]]=map.CornerNormals[f][k];return true;
            }
            if(!Add(seed,true))throw new InvalidDataException("Cannot parameterize a triangle.");
            var frontier=new SortedSet<int>(neighbors[seed]);
            while(frontier.Count>0&&island.Faces.Count<64&&island.StretchV==1) {
                int candidate=frontier.Min;frontier.Remove(candidate);
                if(done.Contains(candidate)||!Add(candidate,false))continue;
                foreach(int f in neighbors[candidate])if(!done.Contains(f))frontier.Add(f);
            }
            result.Add(island);
        }
        return result;
    }

    static List<List<UvIsland>> PackIslands(List<UvIsland> islands)
    {
        // Height-sorted shelves. Integer placement, no scaling to fit a page.
        var pages=new List<List<UvIsland>>();var page=new List<UvIsland>();
        int width=Math.Clamp((int)Math.Ceiling(Math.Sqrt(islands.Sum(i=>(double)i.Width*i.Height))*1.1),256,4096);
        width=Math.Max(width,islands.Max(i=>i.Width));int x=0,y=0,rowHeight=0;
        foreach(var island in islands.OrderByDescending(i=>i.Height).ThenByDescending(i=>i.Width).ThenBy(i=>i.Faces[0])) {
            if(x+island.Width>width){x=0;y+=rowHeight;rowHeight=0;}
            if(y+island.Height>4096&&page.Count>0){pages.Add(page);page=[];x=0;y=0;rowHeight=0;}
            island.X=x;island.Y=y;page.Add(island);x+=island.Width;rowHeight=Math.Max(rowHeight,island.Height);
        }
        if(page.Count>0)pages.Add(page);return pages;
    }
    static List<JsonObject> BakeUvIslands(GlbDocument doc,TextureStore store,SurfaceMap map,int[] faces,int materialId,
        UvReuse reuse,IProgress<WorkProgress>? progress,List<string> reports)
    {
        var material=store.Material(materialId);
        int columns=(int)Math.Ceiling(Math.Sqrt(Math.Min(faces.Length,65536))),size=256;
        while(size<Math.Max(columns*16,Math.Min(4096,material.Size)))size*=2;
        size=Math.Min(size,4096);int referenceCell=size/columns;
        var islands=BuildIslands(map,faces,referenceCell);var pages=PackIslands(islands);var result=new List<JsonObject>();
        int acceptedColor=0,acceptedMetal=0,sourceMaskUsed=0;
        foreach(var page in pages) {
            progress?.Report(new("FA-QEM: baking connected UV islands",0));
            int width=page.Max(i=>i.X+i.Width),height=page.Max(i=>i.Y+i.Height),length=checked(width*height);
            var used=new bool[length];var colors=new byte[length*4];var normalPixels=new byte[length*4];
            var mr=material.MetalRough is null?[]:new byte[length*4];var ao=material.Occlusion is null?[]:new byte[length*4];
            var emission=material.Emissive is null?[]:new byte[length*4];
            int completed=0,normalFallbacks=0;var progressLock=new object();
            Parallel.ForEach(page,new ParallelOptions{MaxDegreeOfParallelism=Math.Max(1,Math.Min(8,Environment.ProcessorCount))},island=> {
                var chart=island.Faces.Select(f=>FaceVertices(map,f).Select(v=>island.Pixel(map.Positions[v])).ToArray()).ToArray();
                int count=island.Width*island.Height;var owners=new int[count];Array.Fill(owners,-1);var distances=new double[count];Array.Fill(distances,double.PositiveInfinity);
                for(int local=0;local<chart.Length;local++) {
                    int face=local;VisitFootprint(chart[local],island.Width,island.Height,(x,y,w,d)=> {
                        int pixel=y*island.Width+x;
                        if(d<distances[pixel]){distances[pixel]=d;owners[pixel]=face;}
                    });
                }
                for(int y=0;y<island.Height;y++)for(int x=0;x<island.Width;x++) {
                    int owner=owners[y*island.Width+x];if(owner<0)continue;
                    int f=island.Faces[owner];int[] vertices=FaceVertices(map,f);P weights=PixelWeights(chart[owner],x+.5,y+.5);
                    Vector4 color=default,metal=default,occlusion=default,emissive=default;P normal=default;
                    for(int sy=0;sy<2;sy++)for(int sx=0;sx<2;sx++) {
                        double px=x+.25+sx*.5,py=y+.25+sy*.5;
                        // Cross shared chart edges during supersampling instead of
                        // dilating each triangle independently into its neighbor.
                        int sampleOwner=owner;P w=PixelWeights(chart[owner],px,py);
                        double best=PixelDistance(chart[owner],w,px,py);
                        if(best>1e-12)for(int neighbor=0;neighbor<chart.Length;neighbor++)if(neighbor!=owner) {
                            P other=PixelWeights(chart[neighbor],px,py);double d=PixelDistance(chart[neighbor],other,px,py);
                            if(d<best){best=d;sampleOwner=neighbor;w=other;if(d<1e-12)break;}
                        }
                        int sampleFace=island.Faces[sampleOwner];var ids=FaceVertices(map,sampleFace);
                        P point=Blend(map.Positions[ids[0]],map.Positions[ids[1]],map.Positions[ids[2]],w);
                        var hit=map.Query(sampleFace,point,FaceNormal(map,sampleFace));var source=map.Source.Faces[hit.Face];
                        var sourceMaterial=store.Material(source.Primitive.Material);
                        Vector4 c=sourceMaterial.Base?.Sample(source,hit.Barycentric,true)??Vector4.One;
                        if(source.Primitive.Colors is float[] vertexColors) {
                            int stride=source.Primitive.ColorSize;
                            Vector4 At(int v)=>new(vertexColors[v*stride],vertexColors[v*stride+1],vertexColors[v*stride+2],stride==4?vertexColors[v*stride+3]:1);
                            c*=At(source.A)*(float)hit.Barycentric.X+At(source.B)*(float)hit.Barycentric.Y+At(source.C)*(float)hit.Barycentric.Z;
                        }
                        color+=new Vector4(c.X*c.W,c.Y*c.W,c.Z*c.W,c.W)*.25f;
                        metal+=(sourceMaterial.MetalRough?.Sample(source,hit.Barycentric,false)??Vector4.One)*.25f;
                        occlusion+=(sourceMaterial.Occlusion?.Sample(source,hit.Barycentric,false)??Vector4.One)*.25f;
                        emissive+=(sourceMaterial.Emissive?.Sample(source,hit.Barycentric,true)??Vector4.One)*.25f;
                        normal+=SourceNormal(source,hit.Barycentric,sourceMaterial);
                        if((reuse.Mask[sampleFace]&1)!=0&&(source.Primitive.Colors is not null||!MatchesBinding(material.Base,source,hit.Barycentric,
                            Blend(reuse.Corners[sampleFace][0],reuse.Corners[sampleFace][1],reuse.Corners[sampleFace][2],w),true,true)))reuse.Mask[sampleFace]&=~1;
                        if((reuse.Mask[sampleFace]&2)!=0&&!MatchesBinding(material.MetalRough,source,hit.Barycentric,
                            Blend(reuse.MetalCorners[sampleFace][0],reuse.MetalCorners[sampleFace][1],reuse.MetalCorners[sampleFace][2],w),false,false))reuse.Mask[sampleFace]&=~2;
                    }
                    if(color.W>0){color.X/=color.W;color.Y/=color.W;color.Z/=color.W;}
                    var ns=map.CornerNormals[f];P n=Blend(ns[0],ns[1],ns[2],weights).Unit;
                    P t=Blend(IslandTangent(island,ns[0]),IslandTangent(island,ns[1]),IslandTangent(island,ns[2]),weights);
                    t=(t-n*n.Dot(t)).Unit;P b=n.Cross(t);normal=normal.Unit;
                    if(n.Length==0)throw new InvalidDataException("Zero interpolated normal in UV island.");
                    if(normal.Length==0){normal=n;Interlocked.Increment(ref normalFallbacks);}
                    var mapped=new Vector4((float)(normal.Dot(t)*.5+.5),(float)(normal.Dot(b)*.5+.5),(float)(normal.Dot(n)*.5+.5),1);
                    int pixel=(island.Y+y)*width+island.X+x,offset=pixel*4;used[pixel]=true;
                    WritePixel(colors,offset,color,true);WritePixel(normalPixels,offset,mapped,false);
                    if(mr.Length>0)WritePixel(mr,offset,metal,false);if(ao.Length>0)WritePixel(ao,offset,occlusion,false);
                    if(emission.Length>0)WritePixel(emission,offset,emissive,true);
                }
                lock(progressLock){completed+=island.Faces.Count;progress?.Report(new("FA-QEM: baking connected UV islands",100*completed/page.Sum(i=>i.Faces.Count)));}
            });
            var allFaces=page.SelectMany(i=>i.Faces).ToArray();
            bool[] FallbackMask(int bit) {
                var mask=new bool[length];
                foreach(var island in page)foreach(int face in island.Faces)if((reuse.Mask[face]&bit)==0) {
                    var triangle=FaceVertices(map,face).Select(v=>island.Pixel(map.Positions[v])).ToArray();
                    VisitFootprint(triangle,island.Width,island.Height,(x,y,_,_)=>mask[(island.Y+y)*width+island.X+x]=true);
                }
                return mask;
            }
            var root=doc.Root;if(root["samplers"] is null)root["samplers"]=new JsonArray();
            int sampler=root["samplers"]!.AsArray().Count;
            root["samplers"]!.AsArray().Add(new JsonObject{["magFilter"]=9729,["minFilter"]=9729,["wrapS"]=33071,["wrapT"]=33071});
            if(root["textures"] is null)root["textures"]=new JsonArray();
            long imageBytes=0;
            (byte[] Bytes,string Mime) EncodeMasked(byte[] pixels,bool[] mask) {
                int first=Array.FindIndex(mask,x=>x);if(first<0)throw new InvalidDataException("Empty atlas footprint.");
                for(int i=0;i<mask.Length;i++)if(!mask[i])for(int k=0;k<4;k++)pixels[i*4+k]=pixels[first*4+k];
                bool constant=true;for(int i=4;i<pixels.Length&&constant;i++)if(pixels[i]!=pixels[i%4])constant=false;
                return Encode(constant?pixels[..4]:pixels,constant?1:width,constant?1:height);
            }
            int Texture((byte[] Bytes,string Mime) encoded) {
                imageBytes+=encoded.Bytes.Length;
                int image=doc.AppendImage(encoded.Bytes,encoded.Mime);var textures=root["textures"]!.AsArray();int id=textures.Count;
                var texture=new JsonObject{["sampler"]=sampler};
                if(encoded.Mime=="image/webp")texture["extensions"]=new JsonObject{["EXT_texture_webp"]=new JsonObject{["source"]=image}};
                else texture["source"]=image;
                textures.Add(texture);return id;
            }
            JsonObject Info(int texture)=>new(){["index"]=texture,["texCoord"]=0};
            progress?.Report(new("FA-QEM: lossless normal atlas encoding"));
            int normalTexture=Texture(EncodeMasked(normalPixels,used));
            var faceIsland=page.SelectMany(i=>i.Faces.Select(f=>(Face:f,Island:i))).ToDictionary(x=>x.Face,x=>x.Island);
            // Compare channel-independent reuse plans, including source image
            // bytes and extra vertex streams. Color fidelity does not force a
            // spatially constant metallic/roughness channel to be rebaked.
            var channels=new Dictionary<string,byte[]>{["color"]=colors};
            if(mr.Length>0)channels["mr"]=mr;if(ao.Length>0)channels["ao"]=ao;if(emission.Length>0)channels["emission"]=emission;
            var fullImages=new Dictionary<string,(byte[] Bytes,string Mime)>();
            foreach(var (name,pixels) in channels){progress?.Report(new("FA-QEM: lossless "+name+" atlas encoding"));fullImages[name]=EncodeMasked(pixels,used);}
            var partialImages=new Dictionary<string,(byte[] Bytes,string Mime)>();int present=0;
            foreach(var (name,bit) in new[]{("color",1),("mr",2)})if(channels.ContainsKey(name)&&allFaces.Any(f=>(reuse.Mask[f]&bit)!=0)) {
                present|=bit;
                if(allFaces.Any(f=>(reuse.Mask[f]&bit)==0)) {
                    progress?.Report(new("FA-QEM: encoding "+name+" fallback regions"));partialImages[name]=EncodeMasked(channels[name],FallbackMask(bit));
                }
            }
            long GeometryBytes(int plan) {
                var vertices=new Dictionary<int,HashSet<(int,int,P,Vector2,Vector2)>>();
                foreach(int f in allFaces) {
                    int mask=reuse.Mask[f]&plan;
                    if(!vertices.TryGetValue(mask,out var set))vertices[mask]=set=[];
                    var ids=FaceVertices(map,f);int chartId=faceIsland[f].Faces[0];
                    for(int k=0;k<3;k++)set.Add((ids[k],chartId,map.CornerNormals[f][k],(mask&1)!=0?reuse.Corners[f][k]:default,(mask&2)!=0?reuse.MetalCorners[f][k]:default));
                }
                return vertices.Sum(g=>(long)g.Value.Count*(48+((g.Key&1)!=0?8:0)+((g.Key&2)!=0?8:0)))+2048L*vertices.Count;
            }
            long fullCost=fullImages.Values.Sum(i=>(long)i.Bytes.Length)+GeometryBytes(0),bestCost=fullCost;int chosen=0;
            for(int plan=1;plan<=3;plan++)if((plan&present)==plan) {
                long cost=GeometryBytes(plan)+OriginalImageCost(doc,material,plan&~sourceMaskUsed);
                foreach(var (name,image) in fullImages) {
                    int bit=name=="color"?1:name=="mr"?2:0;
                    if((plan&bit)==0)cost+=image.Bytes.Length;else if(partialImages.TryGetValue(name,out var partial))cost+=partial.Bytes.Length;
                }
                if(cost<bestCost){bestCost=cost;chosen=plan;}
            }
            sourceMaskUsed|=chosen;
            var textureIndices=new Dictionary<string,int>();
            int Baked(string name) {
                if(textureIndices.TryGetValue(name,out int found))return found;
                int bit=name=="color"?1:name=="mr"?2:0;
                int texture=Texture((chosen&bit)!=0?partialImages[name]:fullImages[name]);textureIndices.Add(name,texture);return texture;
            }
            foreach(var group in allFaces.GroupBy(f=>reuse.Mask[f]&chosen)) {
                int mask=group.Key;int[] subset=group.ToArray();
                var outputMaterial=material.Json.DeepClone().AsObject();outputMaterial["name"]=$"FA-QEM UV islands (source channels {mask})";
                if(outputMaterial["pbrMetallicRoughness"] is null)outputMaterial["pbrMetallicRoughness"]=new JsonObject();
                var pbr=outputMaterial["pbrMetallicRoughness"]!.AsObject();
                void RestoreUv(JsonNode info,int set) {
                    info["texCoord"]=set;
                    if(info["extensions"]?["KHR_texture_transform"] is JsonObject transform)transform["texCoord"]=set;
                }
                if((mask&1)!=0)RestoreUv(pbr["baseColorTexture"]!,1);else pbr["baseColorTexture"]=Info(Baked("color"));
                if((mask&2)!=0)RestoreUv(pbr["metallicRoughnessTexture"]!,2);
                else if(mr.Length>0)pbr["metallicRoughnessTexture"]=Info(Baked("mr"));else pbr.Remove("metallicRoughnessTexture");
                if(ao.Length>0){var info=Info(Baked("ao"));info["strength"]=material.Json["occlusionTexture"]?["strength"]?.GetValue<float>()??1;outputMaterial["occlusionTexture"]=info;}else outputMaterial.Remove("occlusionTexture");
                if(emission.Length>0)outputMaterial["emissiveTexture"]=Info(Baked("emission"));else outputMaterial.Remove("emissiveTexture");
                outputMaterial["normalTexture"]=Info(normalTexture);
                if(root["materials"] is null)root["materials"]=new JsonArray();int materialIndex=root["materials"]!.AsArray().Count;root["materials"]!.AsArray().Add(outputMaterial);
                var positions=new List<float>();var normals=new List<float>();var tangents=new List<float>();var atlasUvs=new List<float>();var sourceUvs=new List<float>();var metalUvs=new List<float>();var indices=new List<uint>();
                var unique=new Dictionary<(int Vertex,int Island,Vector2 Uv,Vector2 Metal,P Normal),uint>();
                foreach(int face in subset) {
                    var island=faceIsland[face];var vertices=FaceVertices(map,face);
                    for(int k=0;k<3;k++) {
                        int v=vertices[k];Vector2 sourceUv=(mask&1)!=0?reuse.Corners[face][k]:default,metalUv=(mask&2)!=0?reuse.MetalCorners[face][k]:default;
                        var key=(v,island.Faces[0],sourceUv,metalUv,map.CornerNormals[face][k]);
                        if(!unique.TryGetValue(key,out uint index)) {
                            index=(uint)(positions.Count/3);unique.Add(key,index);P p=map.Positions[v],n=map.CornerNormals[face][k],t=IslandTangent(island,n);Vector2 uv=island.Pixel(p);
                            positions.AddRange([(float)p.X,(float)p.Y,(float)p.Z]);normals.AddRange([(float)n.X,(float)n.Y,(float)n.Z]);tangents.AddRange([(float)t.X,(float)t.Y,(float)t.Z,1]);
                            atlasUvs.AddRange([(uv.X+island.X)/width,(uv.Y+island.Y)/height]);
                            if((mask&1)!=0)sourceUvs.AddRange([sourceUv.X,sourceUv.Y]);if((mask&2)!=0)metalUvs.AddRange([metalUv.X,metalUv.Y]);
                        }
                        indices.Add(index);
                    }
                }
                var attributes=new JsonObject{["POSITION"]=doc.AppendFloats(positions.ToArray(),3,true),["NORMAL"]=doc.AppendFloats(normals.ToArray(),3,false),["TANGENT"]=doc.AppendFloats(tangents.ToArray(),4,false),["TEXCOORD_0"]=doc.AppendFloats(atlasUvs.ToArray(),2,false)};
                if((mask&1)!=0)attributes["TEXCOORD_1"]=doc.AppendFloats(sourceUvs.ToArray(),2,false);
                if((mask&2)!=0)attributes["TEXCOORD_2"]=doc.AppendFloats(metalUvs.ToArray(),2,false);
                result.Add(new JsonObject{["mode"]=4,["material"]=materialIndex,["attributes"]=attributes,["indices"]=doc.AppendIndices(indices.ToArray())});
            }
            int colorCount=allFaces.Count(f=>(reuse.Mask[f]&chosen&1)!=0),metalCount=allFaces.Count(f=>(reuse.Mask[f]&chosen&2)!=0);
            acceptedColor+=colorCount;acceptedMetal+=metalCount;
            if(normalFallbacks>0)reports.Add($"Normales : {normalFallbacks:N0} texels ont repris la normale du résultat après annulation de la moyenne des normales source.");
            reports.Add($"Îlots UV : {page.Count} îlots pour {allFaces.Length:N0} triangles, atlas {width}×{height}, images recuites {imageBytes/1e6:F3} Mo ; correspondances validées avant comparaison de poids : couleur {allFaces.Count(f=>(reuse.Mask[f]&1)!=0):N0}, métal/rugosité {allFaces.Count(f=>(reuse.Mask[f]&2)!=0):N0} ; réutilisation source : couleur {colorCount:N0}, métal/rugosité {metalCount:N0} triangles. Économie estimée de la réutilisation, images et sommets compris : {(fullCost-bestCost)/1e6:F3} Mo.");
        }
        reports.Add($"Réutilisation UV par canal : couleur {acceptedColor:N0}, métal/rugosité {acceptedMetal:N0}, sur {faces.Length:N0} triangles. Contrôle local : écart de canal ≤ 1/1024 aux échantillons contrôlés, alpha compris ; écart UV couleur ≤ 0,125 texel. Métal/rugosité peut réutiliser une coordonnée constante si les valeurs correspondent. Contrôle échantillonné, pas une preuve d’identité continue ; consulter les mesures finales.");
        return result;
    }
    static long OriginalImageCost(GlbDocument doc,MaterialData material,int mask)
    {
        var pbr=material.Json["pbrMetallicRoughness"];var ids=new HashSet<int>();
        foreach(var info in new[]{(mask&1)!=0?pbr?["baseColorTexture"]:null,(mask&2)!=0?pbr?["metallicRoughnessTexture"]:null})if(info is not null) {
            var texture=doc.Root["textures"]![info["index"]!.GetValue<int>()]!;
            int image=texture["source"]?.GetValue<int>()??texture["extensions"]!["EXT_texture_webp"]!["source"]!.GetValue<int>();ids.Add(image);
        }
        long bytes=0;
        foreach(int id in ids) {
            var image=doc.Root["images"]![id]!;
            if(image["bufferView"] is JsonNode view)bytes+=doc.Root["bufferViews"]![view.GetValue<int>()]!["byteLength"]!.GetValue<int>();
            else bytes+=image["uri"]!.GetValue<string>().Length;
        }
        return bytes;
    }
    static P IslandTangent(UvIsland island,P n)=>(island.U-n*n.Dot(island.U)).Unit;
    static P PixelWeights(Vector2[] triangle,double x,double y)=>ClosestBarycentric(new(x,y,0),new(triangle[0].X,triangle[0].Y,0),new(triangle[1].X,triangle[1].Y,0),new(triangle[2].X,triangle[2].Y,0));
    static double PixelDistance(Vector2[] triangle,P weights,double x,double y)
    {
        // Stay in double precision: a float round-off inside a triangle must not
        // trigger a search through all neighboring triangles for each sub-sample.
        double qx=triangle[0].X*weights.X+triangle[1].X*weights.Y+triangle[2].X*weights.Z;
        double qy=triangle[0].Y*weights.X+triangle[1].Y*weights.Y+triangle[2].Y*weights.Z;
        double dx=x-qx,dy=y-qy;return dx*dx+dy*dy;
    }
    static void VisitFootprint(Vector2[] triangle,int width,int height,Action<int,int,P,double> visit)
    {
        int minX=Math.Max(0,(int)Math.Floor(triangle.Min(p=>p.X)-2)),maxX=Math.Min(width-1,(int)Math.Ceiling(triangle.Max(p=>p.X)+2));
        int minY=Math.Max(0,(int)Math.Floor(triangle.Min(p=>p.Y)-2)),maxY=Math.Min(height-1,(int)Math.Ceiling(triangle.Max(p=>p.Y)+2));
        for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++) {
            P w=PixelWeights(triangle,x+.5,y+.5);double d=PixelDistance(triangle,w,x+.5,y+.5);if(d<=2.01)visit(x,y,w,d);
        }
    }

}
