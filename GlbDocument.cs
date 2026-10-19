using System.Buffers.Binary;
using System.Text;
using System.Text.Json.Nodes;
namespace FaQemSharp;
    public sealed class GlbDocument
    {
        public readonly JsonObject Root;
        readonly byte[] inputBin;
        readonly MemoryStream output=new();
        JsonArray Accessors=>Root["accessors"]!.AsArray();
        JsonArray Views=>Root["bufferViews"]!.AsArray();
        public GlbDocument(byte[] bytes) {
            if(bytes.Length<20||BinaryPrimitives.ReadUInt32LittleEndian(bytes)!=0x46546c67||BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(4))!=2||BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(8))!=bytes.Length)
                throw new InvalidDataException("Invalid GLB 2.0 header.");
            JsonObject? root=null;byte[]? bin=null;
            for(int offset=12;offset<bytes.Length;) {
                if(bytes.Length-offset<8)throw new InvalidDataException("Truncated GLB chunk.");
                int size=checked((int)BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(offset)));
                uint type=BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(offset+4));offset+=8;
                if(size<0||size>bytes.Length-offset)throw new InvalidDataException("Invalid GLB chunk size.");
                if(type==0x4e4f534a)root=JsonNode.Parse(bytes.AsSpan(offset,size))!.AsObject();
                else if(type==0x004e4942)bin=bytes.AsSpan(offset,size).ToArray();
                else throw new InvalidDataException("Unsupported GLB chunk.");
                offset+=size;
            }
            Root=root??throw new InvalidDataException("Missing GLB JSON.");
            inputBin=bin??throw new InvalidDataException("Missing GLB binary buffer.");
            if(Root["buffers"] is not JsonArray buffers||buffers.Count!=1||buffers[0]?["uri"] is not null)
                throw new InvalidDataException("The model must use one embedded GLB buffer.");
            if(Root["meshes"] is not JsonArray||Root["accessors"] is not JsonArray||Root["bufferViews"] is not JsonArray)
                throw new InvalidDataException("Missing GLB geometry.");
            output.Write(inputBin);
        }
        (JsonObject accessor,int count,int components,int component,int size,int offset,int stride) Layout(int index) {
            var a=Accessors[index]!.AsObject();
            if(a["sparse"] is not null)throw new InvalidDataException("Sparse geometry accessors are not supported by this decimation path.");
            var v=Views[a["bufferView"]!.GetValue<int>()]!.AsObject();
            if((v["buffer"]?.GetValue<int>()??0)!=0)throw new InvalidDataException("Geometry uses an external buffer.");
            if(v["extensions"]?["EXT_meshopt_compression"] is not null||v["extensions"]?["KHR_meshopt_compression"] is not null)
                throw new InvalidDataException("Decode meshopt-compressed geometry before decimating this GLB.");
            int count=a["count"]!.GetValue<int>(),component=a["componentType"]!.GetValue<int>();
            int components=a["type"]!.GetValue<string>() switch {"SCALAR"=>1,"VEC2"=>2,"VEC3"=>3,"VEC4"=>4,_=>throw new InvalidDataException("Unsupported geometry accessor type.")};
            int size=component switch {5120 or 5121=>1,5122 or 5123=>2,5125 or 5126=>4,_=>throw new InvalidDataException("Unsupported component type.")};
            int viewOffset=v["byteOffset"]?.GetValue<int>()??0, relative=a["byteOffset"]?.GetValue<int>()??0;
            int stride=v["byteStride"]?.GetValue<int>()??size*components, viewSize=v["byteLength"]!.GetValue<int>();
            long extent=count==0?relative:(long)relative+(long)(count-1)*stride+size*components;
            if(count<0||relative<0||viewOffset<0||stride<size*components||extent>viewSize||(long)viewOffset+viewSize>inputBin.Length)
                throw new InvalidDataException("Geometry accessor exceeds its buffer.");
            return(a,count,components,component,size,checked(viewOffset+relative),stride);
        }
        public float[] ReadFloats(int index,out int components) {
            var l=Layout(index);components=l.components;
            var result=new float[checked(l.count*components)];
            bool normalized=l.accessor["normalized"]?.GetValue<bool>()??false;
            for(int i=0;i<l.count;i++)for(int k=0;k<components;k++) {
                int offset=l.offset+i*l.stride+k*l.size;
                float value=l.component switch {5126=>BitConverter.ToSingle(inputBin,offset),5125=>BinaryPrimitives.ReadUInt32LittleEndian(inputBin.AsSpan(offset)),5123=>BinaryPrimitives.ReadUInt16LittleEndian(inputBin.AsSpan(offset)),5122=>BinaryPrimitives.ReadInt16LittleEndian(inputBin.AsSpan(offset)),5121=>inputBin[offset],5120=>(sbyte)inputBin[offset],_=>0};
                if(normalized)value=l.component switch {5120=>Math.Max(-1,value/127),5121=>value/255,5122=>Math.Max(-1,value/32767),5123=>value/65535,_=>value};
                if(!float.IsFinite(value))throw new InvalidDataException("Non-finite vertex attribute.");
                result[i*components+k]=value;
            }
            return result;
        }
        public uint[] ReadIndices(int index) {
            var l=Layout(index);
            if(l.components!=1||l.component is not (5121 or 5123 or 5125))throw new InvalidDataException("Invalid index accessor.");
            var result=new uint[l.count];
            for(int i=0;i<result.Length;i++) {int offset=l.offset+i*l.stride;result[i]=l.component switch {5121=>inputBin[offset],5123=>BinaryPrimitives.ReadUInt16LittleEndian(inputBin.AsSpan(offset)),_=>BinaryPrimitives.ReadUInt32LittleEndian(inputBin.AsSpan(offset))};}
            return result;
        }
        public byte[] ReadEmbeddedView(int index) {
            var view=Views[index]!.AsObject();
            if((view["buffer"]?.GetValue<int>()??0)!=0)
                throw new InvalidDataException("Expected an embedded GLB buffer.");
            int offset=view["byteOffset"]?.GetValue<int>()??0,length=view["byteLength"]!.GetValue<int>();
            if(offset<0||length<0||(long)offset+length>inputBin.Length)
                throw new InvalidDataException("Invalid embedded GLB buffer view.");
            return inputBin.AsSpan(offset,length).ToArray();
        }
        int AppendView(byte[] bytes,int target) {
            while(output.Length%4!=0)output.WriteByte(0);
            int offset=checked((int)output.Length);output.Write(bytes);
            int view=Views.Count;Views.Add(new JsonObject{["buffer"]=0,["byteOffset"]=offset,["byteLength"]=bytes.Length,["target"]=target});return view;
        }
        public int AppendImage(byte[] png,string mimeType="image/png") {
            int view=AppendView(png,0);Views[view]!.AsObject().Remove("target");
            if(Root["images"] is null)Root["images"]=new JsonArray();
            var images=Root["images"]!.AsArray();int id=images.Count;
            images.Add(new JsonObject{["bufferView"]=view,["mimeType"]=mimeType});return id;
        }
        public int AppendFloats(float[] values,int components,bool position) {
            var bytes=new byte[checked(values.Length*4)];Buffer.BlockCopy(values,0,bytes,0,bytes.Length);
            var accessor=new JsonObject{["bufferView"]=AppendView(bytes,34962),["componentType"]=5126,["count"]=values.Length/components,["type"]=components==1?"SCALAR":"VEC"+components};
            if(position) {
                var min=new JsonArray();var max=new JsonArray();
                for(int k=0;k<components;k++) {float lo=float.MaxValue,hi=float.MinValue;for(int i=k;i<values.Length;i+=components){lo=Math.Min(lo,values[i]);hi=Math.Max(hi,values[i]);}min.Add(lo);max.Add(hi);}
                accessor["min"]=min;accessor["max"]=max;
            }
            int index=Accessors.Count;Accessors.Add(accessor);return index;
        }
        public int AppendIndices(uint[] indices) {
            var bytes=new byte[indices.Length*4];Buffer.BlockCopy(indices,0,bytes,0,bytes.Length);
            int index=Accessors.Count;Accessors.Add(new JsonObject{["bufferView"]=AppendView(bytes,34963),["componentType"]=5125,["count"]=indices.Length,["type"]="SCALAR"});return index;
        }
        public byte[] Serialize() {
            CompactBuffers();
            Root["buffers"]![0]!["byteLength"]=checked((int)output.Length);
            byte[] json=Encoding.UTF8.GetBytes(Root.ToJsonString());
            int jsonSize=(json.Length+3)&~3,binSize=checked(((int)output.Length+3)&~3);
            using var result=new MemoryStream();using var writer=new BinaryWriter(result);
            writer.Write(0x46546c67u);writer.Write(2u);writer.Write(checked(28+jsonSize+binSize));
            writer.Write(jsonSize);writer.Write(0x4e4f534au);writer.Write(json);for(int i=json.Length;i<jsonSize;i++)writer.Write((byte)32);
            writer.Write(binSize);writer.Write(0x004e4942u);writer.Write(output.ToArray());for(long i=output.Length;i<binSize;i++)writer.Write((byte)0);
            return result.ToArray();
        }

        void CompactBuffers() {
            // Follow glTF accessor references before removing the superseded geometry.
            // Extension payloads outside the supported static GLB profile retain their
            // buffers intact rather than guessing at extension-specific references.
            if(Root["extensionsUsed"] is JsonArray extensions && extensions.Any(e=> {
                string name=e!.GetValue<string>();
                return !name.StartsWith("KHR_materials_") && name is not ("EXT_texture_webp" or "KHR_texture_basisu" or "KHR_texture_transform" or "KHR_mesh_quantization" or "KHR_lights_punctual" or "EXT_mesh_gpu_instancing");
            }))return;
            var references=new List<(JsonObject Owner,string Key)>();
            void AccessorReferences(JsonNode? node) {
                if(node is JsonObject obj)foreach(var pair in obj) {
                    if(pair.Key is "accessors" or "bufferViews" or "extras")continue;
                    if(pair.Key=="attributes"&&pair.Value is JsonObject attributes) {
                        foreach(var a in attributes)references.Add((attributes,a.Key));
                    } else if(pair.Key is "indices" or "input" or "output" or "inverseBindMatrices" && pair.Value is JsonValue value && value.TryGetValue<int>(out _))
                        references.Add((obj,pair.Key));
                    else AccessorReferences(pair.Value);
                } else if(node is JsonArray array)foreach(var item in array)AccessorReferences(item);
            }
            AccessorReferences(Root);
            var oldAccessors=Accessors;var newAccessors=new JsonArray();var accessorMap=new Dictionary<int,int>();
            foreach(var (owner,key) in references) {
                int index=owner[key]!.GetValue<int>();
                if(!accessorMap.TryGetValue(index,out int mapped)) {mapped=newAccessors.Count;accessorMap.Add(index,mapped);newAccessors.Add(oldAccessors[index]!.DeepClone());}
                owner[key]=mapped;
            }
            Root["accessors"]=newAccessors;
            references.Clear();
            void ViewReferences(JsonNode? node) {
                if(node is JsonObject obj)foreach(var pair in obj) {
                    if(pair.Key is "bufferViews" or "extras")continue;
                    if(pair.Key=="bufferView"&&pair.Value is JsonValue)references.Add((obj,pair.Key));
                    else ViewReferences(pair.Value);
                } else if(node is JsonArray array)foreach(var item in array)ViewReferences(item);
            }
            ViewReferences(Root);
            var oldViews=Views;var newViews=new JsonArray();var viewMap=new Dictionary<int,int>();
            byte[] oldData=output.ToArray();output.SetLength(0);
            foreach(var (owner,key) in references) {
                int index=owner[key]!.GetValue<int>();
                if(!viewMap.TryGetValue(index,out int mapped)) {
                    var view=oldViews[index]!.DeepClone().AsObject();
                    int offset=view["byteOffset"]?.GetValue<int>()??0,size=view["byteLength"]!.GetValue<int>();
                    if(offset<0||size<0||(long)offset+size>oldData.Length)throw new InvalidDataException("Invalid referenced buffer view.");
                    while(output.Length%4!=0)output.WriteByte(0);
                    view["byteOffset"]=checked((int)output.Length);output.Write(oldData,offset,size);
                    mapped=newViews.Count;viewMap.Add(index,mapped);newViews.Add(view);
                }
                owner[key]=mapped;
            }
            Root["bufferViews"]=newViews;
        }
    }
