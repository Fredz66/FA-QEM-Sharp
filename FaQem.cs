namespace FaQemSharp;

// Independent implementation of arXiv:2605.14029v1, equations 1–11.
// Geometry is independent of render-vertex splits. GlbFaQem bakes the recorded
// reverse-collapse correspondence into connected UV islands, with validated
// source texture reuse where the reduced UV mapping remains accurate.
// Publication and supplement: https://arxiv.org/html/2605.14029v1
// Availability checked 2026-10-18: https://github.com/b22237/FA-QEM contains
// the project website and media; its "View Code (Soon)" points to a placeholder.
// No author implementation was available there to resolve the following choices:
//
//  * Algorithm 1 minimizes Qgf ONLY, then evaluates the boundary area cost.
//    Minimizing Qgf + wArea*QA instead would implement a different algorithm.
//  * Eq. 11: with e=s-r and t=r×s, the swept area is |e×x+t|²/2.
//    Since E^T=-E, expanding |E x+t|² gives the off-diagonal
//    E^T t=-E t, exactly the printed equation. Also,
//    e×r = s×r = -t, so the expression vanishes at both endpoints.
//    The direct cross-product evaluation below avoids forming/cancelling the matrix.
//  * Eq. 7/8 plane vectors are used WITHOUT normalization, literally as printed.
//    The base and tangent planes use unit normals. Consequently the composite
//    metric is scale-dependent; we translate coordinates but never rescale them.
//  * Boundary neighbors are ordered by vertex index (the paper does not specify
//    their order, although swapping them changes Eq. 8). Branches are skipped.
//  * Normals are area-weighted geometric vertex normals, computed before reduction.
//    Qgf is accumulated on contraction, not recomputed from the changing mesh.
//  * QA is evaluated from current incident boundary edges, counted once each.
//    The main text suggests current one-rings; the supplement calls it precomputed.
//  * wUV=5000 is listed but its insertion into the algorithm is unspecified.
//    No UV locking or undocumented UV cost is added to Algorithm 1.
// Optional virtual component edges are not inserted. No global intersection or
// manifold guarantee is implied by the local normal check.
internal static class FaQem
{
    internal readonly record struct Point(double X, double Y, double Z)
    {
        public static Point operator +(Point a, Point b) => new(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
        public static Point operator -(Point a, Point b) => new(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
        public static Point operator *(Point a, double s) => new(a.X*s,a.Y*s,a.Z*s);
        public double Dot(Point b) => X*b.X+Y*b.Y+Z*b.Z;
        public Point Cross(Point b) => new(Y*b.Z-Z*b.Y,Z*b.X-X*b.Z,X*b.Y-Y*b.X);
        public double Length => Math.Sqrt(Dot(this));
        public Point Unit => Length>0 ? this*(1/Length) : default;
        public bool Finite => double.IsFinite(X)&&double.IsFinite(Y)&&double.IsFinite(Z);
    }

    // Symmetric homogeneous quadric, ten independent coefficients, all double.
    struct Quadric
    {
        public double XX,XY,XZ,XW,YY,YZ,YW,ZZ,ZW,WW;
        public void Plane(Point n, double d, double w)
        {
            XX+=w*n.X*n.X; XY+=w*n.X*n.Y; XZ+=w*n.X*n.Z; XW+=w*n.X*d;
            YY+=w*n.Y*n.Y; YZ+=w*n.Y*n.Z; YW+=w*n.Y*d;
            ZZ+=w*n.Z*n.Z; ZW+=w*n.Z*d; WW+=w*d*d;
        }
        public static Quadric operator +(Quadric a, Quadric b) => new() {
            XX=a.XX+b.XX,XY=a.XY+b.XY,XZ=a.XZ+b.XZ,XW=a.XW+b.XW,
            YY=a.YY+b.YY,YZ=a.YZ+b.YZ,YW=a.YW+b.YW,ZZ=a.ZZ+b.ZZ,ZW=a.ZW+b.ZW,WW=a.WW+b.WW };
        public double At(Point p) => p.X*(XX*p.X+2*XY*p.Y+2*XZ*p.Z+2*XW)+
            p.Y*(YY*p.Y+2*YZ*p.Z+2*YW)+p.Z*(ZZ*p.Z+2*ZW)+WW;
        public bool Solve(out Point p)
        {
            // Partial pivoting with a relative pivot cutoff; no absolute mesh-unit threshold.
            Span<double> m=stackalloc double[12] {XX,XY,XZ,-XW,XY,YY,YZ,-YW,XZ,YZ,ZZ,-ZW};
            double scale=Math.Max(Math.Abs(XX),Math.Max(Math.Abs(YY),Math.Abs(ZZ)));
            p=default;
            if(!(scale>0)||!double.IsFinite(scale))return false;
            for(int k=0;k<3;k++) {
                int pivot=k;
                for(int j=k+1;j<3;j++)if(Math.Abs(m[j*4+k])>Math.Abs(m[pivot*4+k]))pivot=j;
                if(Math.Abs(m[pivot*4+k])<=scale*1e-12)return false;
                if(pivot!=k)for(int j=k;j<4;j++)(m[k*4+j],m[pivot*4+j])=(m[pivot*4+j],m[k*4+j]);
                for(int i=k+1;i<3;i++) {
                    double f=m[i*4+k]/m[k*4+k];
                    for(int j=k;j<4;j++)m[i*4+j]-=f*m[k*4+j];
                }
            }
            double z=m[11]/m[10], y=(m[7]-m[6]*z)/m[5], x=(m[3]-m[2]*z-m[1]*y)/m[0];
            p=new(x,y,z);return p.Finite;
        }
    }
    readonly record struct Edge(int A,int B)
    {
        public static Edge Of(int a,int b)=>a<b?new(a,b):new(b,a);
    }
    sealed class Vertex(Point position)
    {
        public Point P=position;
        public Quadric Q;
        public HashSet<int> Faces=[];
        public HashSet<int> Neighbors=[];
        public int Version;
        public bool Dead;
        public int History;
    }
    sealed class Face(int a,int b,int c)
    {
        public int A=a,B=b,C=c;
        public bool Dead;
        public int[] Vertices => [A,B,C];
    }
    readonly record struct Candidate(Edge Edge,int VersionA,int VersionB,Point Position);
    // Immutable binary history: leaves are welded input vertices; each internal
    // node records both pre-collapse positions through its children. Traversing
    // root to leaf reverses precisely the contractions affecting that vertex.
    internal readonly record struct HistoryNode(Point Position,int Left,int Right,int SourceVertex);
    internal sealed record Result(uint[] Indices,int[] SourceFaces,int[] Roots,HistoryNode[] History,string Diagnostics)
    {
        public int MapToSource(int vertex,Point sample)
        {
            int node=Roots[vertex];
            while(History[node].Left>=0) {
                var h=History[node];var l=History[h.Left].Position-sample;var r=History[h.Right].Position-sample;
                node=l.Dot(l)<=r.Dot(r)?h.Left:h.Right;
            }
            return History[node].SourceVertex;
        }
    }

    public static Result Simplify(float[] positions,uint[] indices,int target,IProgress<WorkProgress>? progress)
    {
        if(positions.Length==0||positions.Length%3!=0||indices.Length%3!=0||target<1)
            throw new ArgumentException("FA-QEM: invalid mesh dimensions or triangle target.");
        int count=positions.Length/3;
        if(indices.Any(i=>i>=count))
            throw new ArgumentException("FA-QEM: inconsistent vertex streams.");
        progress?.Report(new("FA-QEM: constructing geometric quadrics",0));
        var v=new Vertex[count];
        var history=new List<HistoryNode>(count);
        Point min=new(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity),
            max=new(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity);
        for(int i=0;i<count;i++) {
            Point p=new(positions[3*i],positions[3*i+1],positions[3*i+2]);
            if(!p.Finite)throw new InvalidDataException("FA-QEM: non-finite position.");
            v[i]=new(p){History=i};history.Add(new(p,-1,-1,i));
            min=new(Math.Min(min.X,p.X),Math.Min(min.Y,p.Y),Math.Min(min.Z,p.Z));
            max=new(Math.Max(max.X,p.X),Math.Max(max.Y,p.Y),Math.Max(max.Z,p.Z));
        }
        // Translate only: preserve published scale-dependent weights while reducing cancellation.
        Point origin=min+(max-min)*0.5;
        double diagonal=(max-min).Length;
        if(!(diagonal>0))throw new InvalidDataException("FA-QEM: mesh has zero spatial extent.");
        foreach(var vertex in v)vertex.P-=origin;
        double minEdge=diagonal*1e-8, minCross=diagonal*diagonal*1e-24;
        var faces=new Face[indices.Length/3];
        var edges=new Dictionary<Edge,int>();
        var normals=new Point[count];
        int live=0,removedDegenerate=0,removedDuplicates=0;
        static (int,int,int) FaceKey(int a,int b,int c)=>a<=b&&a<=c?(a,b,c):b<=c?(b,c,a):(c,a,b);
        var faceKeys=new HashSet<(int,int,int)>();
        void ChangeEdge(int a,int b,int delta) {
            var e=Edge.Of(a,b);edges.TryGetValue(e,out int n);n+=delta;
            if(n==0) {edges.Remove(e);v[a].Neighbors.Remove(b);v[b].Neighbors.Remove(a);}
            else {edges[e]=n;v[a].Neighbors.Add(b);v[b].Neighbors.Add(a);}
        }
        void Attach(int id,Face f,int delta) {
            foreach(int x in f.Vertices) {if(delta>0)v[x].Faces.Add(id);else v[x].Faces.Remove(id);}
            ChangeEdge(f.A,f.B,delta);ChangeEdge(f.B,f.C,delta);ChangeEdge(f.C,f.A,delta);
        }
        for(int i=0;i<faces.Length;i++) {
            int a=checked((int)indices[3*i]), b=checked((int)indices[3*i+1]), c=checked((int)indices[3*i+2]);
            var f=faces[i]=new(a,b,c);
            var cross=(v[b].P-v[a].P).Cross(v[c].P-v[a].P);double twiceArea=cross.Length;
            if(a==b||b==c||a==c||twiceArea<=minCross) {f.Dead=true;removedDegenerate++;continue;}
            if(!faceKeys.Add(FaceKey(a,b,c))) {f.Dead=true;removedDuplicates++;continue;}
            Attach(i,f,1);live++;
            Point n=cross*(1/twiceArea);double d=-n.Dot(v[a].P);
            foreach(int x in f.Vertices) {v[x].Q.Plane(n,d,2/twiceArea);normals[x]+=cross;}
        }
        if(live==0)throw new InvalidDataException("FA-QEM: no non-degenerate triangles.");
        for(int i=0;i<count;i++) {
            var p=v[i].P;var n=normals[i].Unit;v[i].Q.Plane(n,-n.Dot(p),0.01);
            var boundary=v[i].Neighbors.Where(j=>edges[Edge.Of(i,j)]==1).Order().ToArray();
            if(boundary.Length!=2)continue;
            Point p2=v[boundary[0]].P,p3=v[boundary[1]].P,first=p3-p2,second=p3-p*2+p2;
            double length=first.Length;
            if(length<=minEdge)continue;
            double curvature=first.Cross(second).Length/(length*length*length);
            // Equations 7/8 do not normalize these two plane normals.
            Point n1=(p-p2).Cross(p3-p),n2=p-p2;
            v[i].Q.Plane(n1,-n1.Dot(p),500*curvature);
            v[i].Q.Plane(n2,-n2.Dot(p),500*curvature);
        }
        var queue=new PriorityQueue<Candidate,(double Cost,long Serial)>();long serial=0;
        long rejectedFlip=0,rejectedNumeric=0,rejectedConnectivity=0;
        double AreaCost(Edge e,Point p) {
            double cost=0;
            foreach(int r in new[]{e.A,e.B})foreach(int s in v[r].Neighbors) {
                if(edges[Edge.Of(r,s)]!=1)continue;
                if(r==e.B&&s==e.A)continue; // one quadric per undirected boundary edge
                // Translation-invariant evaluation of Eq. 11: |e×p+t|²/2.
                var sweep=(v[s].P-v[r].P).Cross(p-v[r].P);
                cost+=0.5*sweep.Dot(sweep);
            }
            return cost;
        }
        void Enqueue(Edge e) {
            var a=v[e.A];var b=v[e.B];
            if(a.Dead||b.Dead||!edges.ContainsKey(e))return;
            if((a.P-b.P).Length<=minEdge) {rejectedNumeric++;return;}
            Quadric q=a.Q+b.Q;
            if(!q.Solve(out Point p)) {
                p=a.P;double best=q.At(p);
                foreach(Point option in new[]{b.P,(a.P+b.P)*0.5}) {
                    double error=q.At(option);if(error<best){best=error;p=option;}
                }
            }
            // Validate exactly the float position that will be written to the GLB,
            // including precision loss when model coordinates have a large offset.
            var world=p+origin;
            p=new Point((float)world.X,(float)world.Y,(float)world.Z)-origin;
            double cost=q.At(p)+100*AreaCost(e,p);
            if(!p.Finite||!double.IsFinite(cost)) {rejectedNumeric++;return;}
            queue.Enqueue(new(e,a.Version,b.Version,p),(Math.Max(0,cost),serial++));
        }
        foreach(var edge in edges.Keys.OrderBy(e=>e.A).ThenBy(e=>e.B))Enqueue(edge);
        int initial=live,collapses=0;
        while(live>target&&queue.TryDequeue(out var entry,out _)) {
            var e=entry.Edge;var a=v[e.A];var b=v[e.B];
            if(a.Dead||b.Dead||entry.VersionA!=a.Version||entry.VersionB!=b.Version||!edges.ContainsKey(e))continue;
            var incident=a.Faces.Concat(b.Faces).Distinct().Order().ToArray();
            bool valid=true;
            foreach(int id in incident) {
                var f=faces[id];var ids=f.Vertices;
                if(ids.Contains(e.A)&&ids.Contains(e.B))continue;
                Point p0=v[f.A].P,p1=v[f.B].P,p2=v[f.C].P;
                Point oldNormal=(p1-p0).Cross(p2-p0);
                if(f.A==e.A||f.A==e.B)p0=entry.Position;
                if(f.B==e.A||f.B==e.B)p1=entry.Position;
                if(f.C==e.A||f.C==e.B)p2=entry.Position;
                Point newNormal=(p1-p0).Cross(p2-p0);
                if(newNormal.Length<=minCross||oldNormal.Dot(newNormal)<=0) {valid=false;break;}
            }
            if(!valid) {rejectedFlip++;continue;}
            // A collapse removes the triangles containing the edge. Their third
            // vertices must not lose their last connection to the surviving star.
            // This is a connectivity guard, not a manifold link condition or UV lock.
            var survivingNeighbors=new HashSet<int>();
            foreach(int id in incident) {
                var ids=faces[id].Vertices;
                if(ids.Contains(e.A)&&ids.Contains(e.B))continue;
                foreach(int x in ids)if(x!=e.A&&x!=e.B)survivingNeighbors.Add(x);
            }
            foreach(int id in incident) {
                var ids=faces[id].Vertices;
                if(!ids.Contains(e.A)||!ids.Contains(e.B))continue;
                foreach(int x in ids)if(x!=e.A&&x!=e.B&&!survivingNeighbors.Contains(x)&&
                    v[x].Faces.Any(j=>!incident.Contains(j)))valid=false;
            }
            // Do not erase an entire source component.
            if(!valid||survivingNeighbors.Count==0) {rejectedConnectivity++;continue;}
            var affected=new HashSet<int>(a.Neighbors.Concat(b.Neighbors)){e.A,e.B};
            history.Add(new(entry.Position+origin,a.History,b.History,-1));a.History=history.Count-1;
            foreach(int id in incident) {var f=faces[id];faceKeys.Remove(FaceKey(f.A,f.B,f.C));Attach(id,f,-1);}
            a.P=entry.Position;a.Q+=b.Q;b.Dead=true;
            foreach(int id in incident) {
                var f=faces[id];
                if(f.A==e.B)f.A=e.A;if(f.B==e.B)f.B=e.A;if(f.C==e.B)f.C=e.A;
                if(f.A==f.B||f.B==f.C||f.A==f.C) {f.Dead=true;live--;}
                else if(!faceKeys.Add(FaceKey(f.A,f.B,f.C))) {f.Dead=true;live--;removedDuplicates++;}
                else Attach(id,f,1);
            }
            // Area quadrics and flip validation depend on the entire changed one-ring.
            foreach(int x in affected)v[x].Version++;
            var updates=new HashSet<Edge>();
            foreach(int x in affected)if(!v[x].Dead)foreach(int y in v[x].Neighbors)updates.Add(Edge.Of(x,y));
            foreach(var edge in updates.OrderBy(e=>e.A).ThenBy(e=>e.B))Enqueue(edge);
            collapses++;
            if(collapses%256==0)progress?.Report(new($"FA-QEM: {live:N0} triangles",(int)(100L*(initial-live)/Math.Max(1,initial-target))));
            // Bound obsolete queue entries without changing the metric or ordering policy.
            if(queue.Count>Math.Max(4096,edges.Count*6L)) {
                queue.Clear();foreach(var edge in edges.Keys.OrderBy(e=>e.A).ThenBy(e=>e.B))Enqueue(edge);
            }
        }
        for(int i=0;i<count;i++)if(!v[i].Dead) {
            Point p=v[i].P+origin;
            if(Math.Abs(p.X)>float.MaxValue||Math.Abs(p.Y)>float.MaxValue||Math.Abs(p.Z)>float.MaxValue)
                throw new InvalidDataException("FA-QEM: output position exceeds GLB float range.");
            positions[3*i]=(float)p.X;positions[3*i+1]=(float)p.Y;positions[3*i+2]=(float)p.Z;
        }
        var result=new List<uint>(live*3);
        var sourceFaces=new List<int>(live);
        // Float output can make otherwise distinct vertex IDs coincide. Deduplicate
        // on the serialized positions as well, keeping opposite-facing surfaces.
        var outputVertices=new Dictionary<Point,int>();var outputFaces=new HashSet<(int,int,int)>();
        int OutputVertex(int x) {
            var p=new Point(positions[3*x],positions[3*x+1],positions[3*x+2]);
            if(!outputVertices.TryGetValue(p,out int id)){id=outputVertices.Count;outputVertices.Add(p,id);}return id;
        }
        for(int id=0;id<faces.Length;id++) {
            var f=faces[id];if(f.Dead)continue;
            if(!outputFaces.Add(FaceKey(OutputVertex(f.A),OutputVertex(f.B),OutputVertex(f.C)))){removedDuplicates++;live--;continue;}
            result.Add((uint)f.A);result.Add((uint)f.B);result.Add((uint)f.C);sourceFaces.Add(id);
        }
        string diagnostics=$"FA-QEM : {collapses:N0} contractions, {live:N0}/{target:N0} triangles. "+
            $"Tentatives refusées (avec réévaluations) : orientation/dégénérescence {rejectedFlip:N0}, calcul numérique {rejectedNumeric:N0}, connexité {rejectedConnectivity:N0}. "+
            $"Doublons orientés retirés : {removedDuplicates:N0}. Triangles initialement dégénérés retirés : {removedDegenerate:N0}. Aucun verrou UV, de matériau ou de bord ouvert.";
        return new(result.ToArray(),sourceFaces.ToArray(),v.Select(x=>x.History).ToArray(),history.ToArray(),diagnostics);
    }
}
