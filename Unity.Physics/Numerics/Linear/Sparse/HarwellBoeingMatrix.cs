using System.Globalization;
using System.IO;
using System.Linq;
using Unity.Collections;
using Unity.Numerics.Linear.Sparse.Primitives;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.AssetImporters;
#endif
using UnityEngine;

namespace Unity.Numerics.Linear.Sparse
{
    internal class HarwellBoeingMatrix : ScriptableObject
    {
        public int NumRows;
        public int NumCols;

        public int[] ColPtrs;
        public int[] RowIndices;
        public float[] Values;

#if UNITY_EDITOR
        public static Matrix LoadFromPath(string path, Allocator alloc)
        {
            var hb = (HarwellBoeingMatrix)AssetDatabase.LoadAssetAtPath(path, typeof(HarwellBoeingMatrix));
            var ptrs = new NativeArray<int>(hb.ColPtrs, Allocator.Temp);
            var idxs = new NativeArray<int>(hb.RowIndices, Allocator.Temp);
            var vals = new NativeArray<float>(hb.Values, Allocator.Temp);

            var m = Matrix.Create(hb.NumRows, hb.NumCols, ptrs, idxs, vals, alloc);

            ptrs.Dispose();
            idxs.Dispose();
            vals.Dispose();

            return m;
        }

#endif
    }

#if UNITY_EDITOR
    [ScriptedImporter(1, "hb")]
    internal class HarwellBoeingImporter : ScriptedImporter
    {
        public override void OnImportAsset(AssetImportContext ctx)
        {
            var lines = File.ReadAllText(ctx.assetPath).Split('\n');

            var mxtype = lines[2].Substring(0, 3);
            if (mxtype != "RUA")
            {
                throw new FileLoadException("Unsupported matrix type!");
            }
            int numRows = int.Parse(lines[2].Substring(14, 14));
            int numCols = int.Parse(lines[2].Substring(28, 14));
            int nonzero = int.Parse(lines[2].Substring(42, 14));

            var obj = ScriptableObject.CreateInstance<HarwellBoeingMatrix>();
            obj.NumRows = numRows;
            obj.NumCols = numCols;

            var data = string.Concat(lines.Skip(4)).Trim(' ', '\n');
            var colPtrs = new int[numCols + 1];
            for (int i = 0; i < numCols + 1; i++)
            {
                var field = new string(data.TakeWhile(char.IsDigit).ToArray());
                colPtrs[i] = int.Parse(field) - 1;
                data = data.Substring(field.Length).Trim(' ', '\n');
            }

            var rowIndices = new int[numRows];
            for (int i = 0; i < numRows; i++)
            {
                var field = new string(data.TakeWhile(char.IsDigit).ToArray());
                rowIndices[i] = int.Parse(field) - 1;
                data = data.Substring(field.Length).Trim(' ', '\n');
            }

            var values = new float[numRows];
            for (int i = 0; i < numRows; i++)
            {
                var field = new string(data.TakeWhile(c => !char.IsWhiteSpace(c)).ToArray());
                values[i] = float.Parse(field, CultureInfo.InvariantCulture);
                data = data.Substring(field.Length).Trim(' ', '\n');
            }

            obj.ColPtrs = colPtrs;
            obj.RowIndices = rowIndices;
            obj.Values = values;

            ctx.AddObjectToAsset("Matrix", obj);
            ctx.SetMainObject(obj);
        }
    }
#endif
}
