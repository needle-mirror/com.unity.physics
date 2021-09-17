using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.IO;
using System;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Physics.Authoring; //for meshes
using UnityEditor;

namespace Unity.DebugDisplay
{
    internal sealed class Context
    {
        internal Context()
        {
        }
    }

    internal struct EightK
    {
        internal FixedString4096Bytes a, b;
    }

    internal struct Unmanaged
    {
        internal void Clear()
        {
            m_TextBuffer.ClearCells();
            m_TextBufferAllocations = m_TextBuffer.AllocateAll();      // clear out all the tex boxes
            m_GraphBufferAllocations = m_GraphBuffer.AllocateAll();  // clear out all the graphs
            m_LineBufferAllocations = m_LineBuffer.AllocateAll(); // clear out all the lines
        }

        internal Unit       m_TextBufferAllocations;
        internal Unit       m_LineBufferAllocations;
        internal Unit       m_GraphBufferAllocations;
        internal Unit       m_GraphDataBufferAllocations;

        internal TextBuffer  m_TextBuffer;
        internal GraphBuffer m_GraphBuffer;
        internal LineBuffer  m_LineBuffer;
        internal LogBuffer   m_LogBuffer;
        internal EightK      m_EightK;

        internal const int kMaxColors = 256;

        internal bool initialized;

        internal UnsafeArray<float4> m_ColorData;

        unsafe internal void Initialize()
        {
            if (initialized == false)
            {
                var pal = stackalloc byte[256 * 3]
                {
                    0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0x00, 0xaa, 0xaa,
                    0xaa, 0x00, 0x00, 0xaa, 0x00, 0xaa, 0xaa, 0x55, 0x00, 0xaa, 0xaa, 0xaa,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0xff, 0x55, 0xff, 0x55, 0x55, 0xff, 0xff,
                    0xff, 0x55, 0x55, 0xff, 0x55, 0xff, 0xff, 0xff, 0x55, 0xff, 0xff, 0xff,
                    0x00, 0x00, 0x00, 0x14, 0x14, 0x14, 0x20, 0x20, 0x20, 0x2c, 0x2c, 0x2c,
                    0x38, 0x38, 0x38, 0x45, 0x45, 0x45, 0x51, 0x51, 0x51, 0x61, 0x61, 0x61,
                    0x71, 0x71, 0x71, 0x82, 0x82, 0x82, 0x92, 0x92, 0x92, 0xa2, 0xa2, 0xa2,
                    0xb6, 0xb6, 0xb6, 0xcb, 0xcb, 0xcb, 0xe3, 0xe3, 0xe3, 0xff, 0xff, 0xff,
                    0x00, 0x00, 0xff, 0x41, 0x00, 0xff, 0x7d, 0x00, 0xff, 0xbe, 0x00, 0xff,
                    0xff, 0x00, 0xff, 0xff, 0x00, 0xbe, 0xff, 0x00, 0x7d, 0xff, 0x00, 0x41,
                    0xff, 0x00, 0x00, 0xff, 0x41, 0x00, 0xff, 0x7d, 0x00, 0xff, 0xbe, 0x00,
                    0xff, 0xff, 0x00, 0xbe, 0xff, 0x00, 0x7d, 0xff, 0x00, 0x41, 0xff, 0x00,
                    0x00, 0xff, 0x00, 0x00, 0xff, 0x41, 0x00, 0xff, 0x7d, 0x00, 0xff, 0xbe,
                    0x00, 0xff, 0xff, 0x00, 0xbe, 0xff, 0x00, 0x7d, 0xff, 0x00, 0x41, 0xff,
                    0x7d, 0x7d, 0xff, 0x9e, 0x7d, 0xff, 0xbe, 0x7d, 0xff, 0xdf, 0x7d, 0xff,
                    0xff, 0x7d, 0xff, 0xff, 0x7d, 0xdf, 0xff, 0x7d, 0xbe, 0xff, 0x7d, 0x9e,
                    0xff, 0x7d, 0x7d, 0xff, 0x9e, 0x7d, 0xff, 0xbe, 0x7d, 0xff, 0xdf, 0x7d,
                    0xff, 0xff, 0x7d, 0xdf, 0xff, 0x7d, 0xbe, 0xff, 0x7d, 0x9e, 0xff, 0x7d,
                    0x7d, 0xff, 0x7d, 0x7d, 0xff, 0x9e, 0x7d, 0xff, 0xbe, 0x7d, 0xff, 0xdf,
                    0x7d, 0xff, 0xff, 0x7d, 0xdf, 0xff, 0x7d, 0xbe, 0xff, 0x7d, 0x9e, 0xff,
                    0xb6, 0xb6, 0xff, 0xc7, 0xb6, 0xff, 0xdb, 0xb6, 0xff, 0xeb, 0xb6, 0xff,
                    0xff, 0xb6, 0xff, 0xff, 0xb6, 0xeb, 0xff, 0xb6, 0xdb, 0xff, 0xb6, 0xc7,
                    0xff, 0xb6, 0xb6, 0xff, 0xc7, 0xb6, 0xff, 0xdb, 0xb6, 0xff, 0xeb, 0xb6,
                    0xff, 0xff, 0xb6, 0xeb, 0xff, 0xb6, 0xdb, 0xff, 0xb6, 0xc7, 0xff, 0xb6,
                    0xb6, 0xdf, 0xb6, 0xb6, 0xff, 0xc7, 0xb6, 0xff, 0xdb, 0xb6, 0xff, 0xeb,
                    0xb6, 0xff, 0xff, 0xb6, 0xeb, 0xff, 0xb6, 0xdb, 0xff, 0xb6, 0xc7, 0xff,
                    0x00, 0x00, 0x71, 0x1c, 0x00, 0x71, 0x38, 0x00, 0x71, 0x55, 0x00, 0x71,
                    0x71, 0x00, 0x71, 0x71, 0x00, 0x55, 0x71, 0x00, 0x38, 0x71, 0x00, 0x1c,
                    0x71, 0x00, 0x00, 0x71, 0x1c, 0x00, 0x71, 0x38, 0x00, 0x71, 0x55, 0x00,
                    0x71, 0x71, 0x00, 0x55, 0x71, 0x00, 0x38, 0x71, 0x00, 0x1c, 0x71, 0x00,
                    0x00, 0x71, 0x00, 0x00, 0x71, 0x1c, 0x00, 0x71, 0x38, 0x00, 0x71, 0x55,
                    0x00, 0x71, 0x71, 0x00, 0x55, 0x71, 0x00, 0x38, 0x71, 0x00, 0x1c, 0x71,
                    0x38, 0x38, 0x71, 0x45, 0x38, 0x71, 0x55, 0x38, 0x71, 0x61, 0x38, 0x71,
                    0x71, 0x38, 0x71, 0x71, 0x38, 0x61, 0x71, 0x38, 0x55, 0x71, 0x38, 0x45,
                    0x71, 0x38, 0x38, 0x71, 0x45, 0x38, 0x71, 0x55, 0x38, 0x71, 0x61, 0x38,
                    0x71, 0x71, 0x38, 0x61, 0x71, 0x38, 0x55, 0x71, 0x38, 0x45, 0x71, 0x38,
                    0x38, 0x71, 0x38, 0x38, 0x71, 0x45, 0x38, 0x71, 0x55, 0x38, 0x71, 0x61,
                    0x38, 0x71, 0x71, 0x38, 0x61, 0x71, 0x38, 0x55, 0x71, 0x38, 0x45, 0x71,
                    0x51, 0x51, 0x71, 0x59, 0x51, 0x71, 0x61, 0x51, 0x71, 0x69, 0x51, 0x71,
                    0x71, 0x51, 0x71, 0x71, 0x51, 0x69, 0x71, 0x51, 0x61, 0x71, 0x51, 0x59,
                    0x71, 0x51, 0x51, 0x71, 0x59, 0x51, 0x71, 0x61, 0x51, 0x71, 0x69, 0x51,
                    0x71, 0x71, 0x51, 0x69, 0x71, 0x51, 0x61, 0x71, 0x51, 0x59, 0x71, 0x51,
                    0x51, 0x71, 0x51, 0x51, 0x71, 0x59, 0x51, 0x71, 0x61, 0x51, 0x71, 0x69,
                    0x51, 0x71, 0x71, 0x51, 0x69, 0x71, 0x51, 0x61, 0x71, 0x51, 0x59, 0x71,
                    0x00, 0x00, 0x41, 0x10, 0x00, 0x41, 0x20, 0x00, 0x41, 0x30, 0x00, 0x41,
                    0x41, 0x00, 0x41, 0x41, 0x00, 0x30, 0x41, 0x00, 0x20, 0x41, 0x00, 0x10,
                    0x41, 0x00, 0x00, 0x41, 0x10, 0x00, 0x41, 0x20, 0x00, 0x41, 0x30, 0x00,
                    0x41, 0x41, 0x00, 0x30, 0x41, 0x00, 0x20, 0x41, 0x00, 0x10, 0x41, 0x00,
                    0x00, 0x41, 0x00, 0x00, 0x41, 0x10, 0x00, 0x41, 0x20, 0x00, 0x41, 0x30,
                    0x00, 0x41, 0x41, 0x00, 0x30, 0x41, 0x00, 0x20, 0x41, 0x00, 0x10, 0x41,
                    0x20, 0x20, 0x41, 0x28, 0x20, 0x41, 0x30, 0x20, 0x41, 0x38, 0x20, 0x41,
                    0x41, 0x20, 0x41, 0x41, 0x20, 0x38, 0x41, 0x20, 0x30, 0x41, 0x20, 0x28,
                    0x41, 0x20, 0x20, 0x41, 0x28, 0x20, 0x41, 0x30, 0x20, 0x41, 0x38, 0x20,
                    0x41, 0x41, 0x20, 0x38, 0x41, 0x20, 0x30, 0x41, 0x20, 0x28, 0x41, 0x20,
                    0x20, 0x41, 0x20, 0x20, 0x41, 0x28, 0x20, 0x41, 0x30, 0x20, 0x41, 0x38,
                    0x20, 0x41, 0x41, 0x20, 0x38, 0x41, 0x20, 0x30, 0x41, 0x20, 0x28, 0x41,
                    0x2c, 0x2c, 0x41, 0x30, 0x2c, 0x41, 0x34, 0x2c, 0x41, 0x3c, 0x2c, 0x41,
                    0x41, 0x2c, 0x41, 0x41, 0x2c, 0x3c, 0x41, 0x2c, 0x34, 0x41, 0x2c, 0x30,
                    0x41, 0x2c, 0x2c, 0x41, 0x30, 0x2c, 0x41, 0x34, 0x2c, 0x41, 0x3c, 0x2c,
                    0x41, 0x41, 0x2c, 0x3c, 0x41, 0x2c, 0x34, 0x41, 0x2c, 0x30, 0x41, 0x2c,
                    0x2c, 0x41, 0x2c, 0x2c, 0x41, 0x30, 0x2c, 0x41, 0x34, 0x2c, 0x41, 0x3c,
                    0x2c, 0x41, 0x41, 0x2c, 0x3c, 0x41, 0x2c, 0x34, 0x41, 0x2c, 0x30, 0x41,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };
                m_ColorData = new UnsafeArray<float4>(kMaxColors);
                for (var i = 0; i < kMaxColors; ++i)
                    m_ColorData[i] = new float4(pal[i * 3 + 2],  pal[i * 3 + 1], pal[i * 3 + 0], 255) / 255.0f;
                m_TextBuffer.Initialize();
                m_GraphBuffer.Initialize();
                m_LineBuffer.Initialize();
                m_LogBuffer = new LogBuffer(new int2(128, 4096));
                m_LogBuffer.m_Fg = ColorIndex.Yellow;
                m_LogBuffer.m_Bg = ColorIndex.Blue;
                m_TextBufferAllocations = m_TextBuffer.AllocateAll();
                m_GraphBufferAllocations = m_GraphBuffer.AllocateAll();
                m_GraphDataBufferAllocations = m_GraphBuffer.ReserveAllData();
                m_LineBufferAllocations = m_LineBuffer.AllocateAll();
                initialized = true;
            }
        }

        internal void Dispose()
        {
            if (initialized == true)
            {
                m_LogBuffer.Dispose();
                m_LineBuffer.Dispose();
                m_GraphBuffer.Dispose();
                m_TextBuffer.Dispose();
                m_ColorData.Dispose();
                initialized = false;
            }
        }

        internal static readonly SharedStatic<Unmanaged> Instance = SharedStatic<Unmanaged>.GetOrCreate<Context, Unmanaged>();
    }


    internal class Managed : IDisposable
    {
        internal struct Objects
        {
            internal Material textMaterial;
            internal Material graphMaterial;
            internal Material lineMaterial;
            internal TextAsset wide;
        }
        internal Objects resources;

        internal static int PixelsWide => Screen.width;
        internal static int PixelsTall => Screen.height;

        internal static float FractionalCellsWide => (float)PixelsWide / Cell.kPixelsWide;
        internal static float FractionalCellsTall => (float)PixelsTall / Cell.kPixelsTall;
        internal static int IntegralCellsWide => (PixelsWide + Cell.kPixelsWide - 1) / Cell.kPixelsWide;
        internal static int IntegralCellsTall => (PixelsTall + Cell.kPixelsTall - 1) / Cell.kPixelsTall;

#if UNITY_EDITOR
        internal static string debugDirName =
            "Packages/com.unity.physics/Unity.Physics.Hybrid/Assets/DebugDisplay/DebugDisplayResources/";
        private static Material debugTextMaterial;
        private static Material graphMaterial;
        private static Material lineMaterial;
        private static TextAsset wideTestAsset;
        private bool resourcesLoaded = false;
#endif

        private void LoadDebugResources()
        {
#if UNITY_EDITOR
            debugTextMaterial = AssetDatabase.LoadAssetAtPath<Material>(Path.Combine(debugDirName, "DebugTextMaterial.mat"));
            graphMaterial = AssetDatabase.LoadAssetAtPath<Material>(Path.Combine(debugDirName, "GraphMaterial.mat"));
            lineMaterial = AssetDatabase.LoadAssetAtPath<Material>(Path.Combine(debugDirName, "LineMaterial.mat"));
            wideTestAsset = AssetDatabase.LoadAssetAtPath<TextAsset>(Path.Combine(debugDirName, "wide.bytes"));
            resourcesLoaded = true;
#endif
        }

        internal unsafe Managed()
        {
            Unmanaged.Instance.Data.Initialize();
#if UNITY_EDITOR
            if (!resourcesLoaded) LoadDebugResources();
            resources.textMaterial = debugTextMaterial;
            resources.graphMaterial = graphMaterial;
            resources.lineMaterial = lineMaterial;
            resources.wide = wideTestAsset;

            Stream s = new MemoryStream(resources.wide.bytes);
            BinaryReader br = new BinaryReader(s);
            var wide = new byte[s.Length];
            br.Read(wide, 0, (int)s.Length);
            fixed(byte* w = wide)
            fixed(EightK * e = &Unmanaged.Instance.Data.m_EightK)
            {
                UnsafeUtility.MemCpy(e, w, 8192);
            }
#endif
#if !UNITY_DOTSRUNTIME
            AppDomain.CurrentDomain.DomainUnload += OnDomainUnload;
#endif
        }

        static void OnDomainUnload(object sender, EventArgs e)
        {
            instance?.Dispose();
        }

        internal void CopyFromCpuToGpu()
        {
            // Recreate compute buffer if needed.
            if (m_ColorBuffer == null || m_ColorBuffer.count != Unmanaged.Instance.Data.m_ColorData.Length)
            {
                if (m_ColorBuffer != null)
                {
                    m_ColorBuffer.Release();
                    m_ColorBuffer = null;
                }

                m_ColorBuffer = new ComputeBuffer(Unmanaged.Instance.Data.m_ColorData.Length, UnsafeUtility.SizeOf<float4>());
                resources.textMaterial.SetBuffer("colorBuffer", m_ColorBuffer);
                resources.graphMaterial.SetBuffer("colorBuffer", m_ColorBuffer);
                resources.lineMaterial.SetBuffer("colorBuffer", m_ColorBuffer);
            }

            if (m_TextCellBuffer == null || m_TextCellBuffer.count != Unmanaged.Instance.Data.m_TextBuffer.m_Screen.m_Cell.Length)
            {
                if (m_TextCellBuffer != null)
                {
                    m_TextCellBuffer.Release();
                    m_TextCellBuffer = null;
                }

                m_TextCellBuffer = new ComputeBuffer(Unmanaged.Instance.Data.m_TextBuffer.m_Screen.m_Cell.Length,
                    UnsafeUtility.SizeOf<Cell>());
                resources.textMaterial.SetBuffer("textBuffer", m_TextCellBuffer);
            }

            if (m_TextInstanceBuffer == null || m_TextInstanceBuffer.count != Unmanaged.Instance.Data.m_TextBuffer.m_Instance.Length)
            {
                if (m_TextInstanceBuffer != null)
                {
                    m_TextInstanceBuffer.Release();
                    m_TextInstanceBuffer = null;
                }

                m_TextInstanceBuffer = new ComputeBuffer(Unmanaged.Instance.Data.m_TextBuffer.m_Instance.Length,
                    UnsafeUtility.SizeOf<TextBuffer.Instance>());
                resources.textMaterial.SetBuffer("positionBuffer", m_TextInstanceBuffer);
            }

            if (m_LineVertexBuffer == null || m_LineVertexBuffer.count != Unmanaged.Instance.Data.m_LineBuffer.m_Instance.Length)
            {
                if (m_LineVertexBuffer != null)
                {
                    m_LineVertexBuffer.Release();
                    m_LineVertexBuffer = null;
                }

                m_LineVertexBuffer = new ComputeBuffer(Unmanaged.Instance.Data.m_LineBuffer.m_Instance.Length, UnsafeUtility.SizeOf<LineBuffer.Instance>());
                resources.lineMaterial.SetBuffer("positionBuffer", m_LineVertexBuffer);
            }

            if (m_GraphSampleBuffer == null || m_GraphSampleBuffer.count != Unmanaged.Instance.Data.m_GraphBuffer.m_Data.Length)
            {
                if (m_GraphSampleBuffer != null)
                {
                    m_GraphSampleBuffer.Release();
                    m_GraphSampleBuffer = null;
                }

                m_GraphSampleBuffer =
                    new ComputeBuffer(Unmanaged.Instance.Data.m_GraphBuffer.m_Data.Length, UnsafeUtility.SizeOf<float>());
                resources.graphMaterial.SetBuffer("sampleBuffer", m_GraphSampleBuffer);
            }

            if (m_GraphInstanceBuffer == null ||
                m_GraphInstanceBuffer.count != Unmanaged.Instance.Data.m_GraphBuffer.m_Instance.Length)
            {
                if (m_GraphInstanceBuffer != null)
                {
                    m_GraphInstanceBuffer.Release();
                    m_GraphInstanceBuffer = null;
                }

                m_GraphInstanceBuffer = new ComputeBuffer(Unmanaged.Instance.Data.m_GraphBuffer.m_Instance.Length,
                    UnsafeUtility.SizeOf<GraphBuffer.Instance>());
                resources.graphMaterial.SetBuffer("instanceBuffer", m_GraphInstanceBuffer);
            }

            m_ColorBuffer.SetData(Unmanaged.Instance.Data.m_ColorData.ToNativeArray());
            m_TextCellBuffer.SetData(Unmanaged.Instance.Data.m_TextBuffer.m_Screen.m_Cell.ToNativeArray());
            m_GraphSampleBuffer.SetData(Unmanaged.Instance.Data.m_GraphBuffer.m_Data.ToNativeArray());

            m_NumGraphsToDraw = Unmanaged.Instance.Data.m_GraphBufferAllocations.Filled;
            m_NumTextBoxesToDraw = Unmanaged.Instance.Data.m_TextBufferAllocations.Filled;
            m_NumLinesToDraw = Unmanaged.Instance.Data.m_LineBufferAllocations.Filled;

            m_GraphInstanceBuffer.SetData(Unmanaged.Instance.Data.m_GraphBuffer.m_Instance.ToNativeArray(), 0, 0, m_NumGraphsToDraw);
            m_TextInstanceBuffer.SetData(Unmanaged.Instance.Data.m_TextBuffer.m_Instance.ToNativeArray(), 0, 0, m_NumTextBoxesToDraw);
            m_LineVertexBuffer.SetData(Unmanaged.Instance.Data.m_LineBuffer.m_Instance.ToNativeArray(), 0, 0, m_NumLinesToDraw);

            var scales = new float4(1.0f / FractionalCellsWide, 1.0f / FractionalCellsTall, 1.0f / PixelsWide, 1.0f / PixelsTall);
            resources.textMaterial.SetVector("scales", scales);
            resources.graphMaterial.SetVector("scales", scales);
            resources.lineMaterial.SetVector("scales", scales);
        }

        internal void Clear()
        {
            Unmanaged.Instance.Data.Clear();
            TextBox.Draw(new int2(0, 0), new int2(IntegralCellsWide, IntegralCellsTall));  // steal one text box for the full-screen thing
        }

        internal void Render()
        {
            resources.lineMaterial.SetPass(0);
            Graphics.DrawProceduralNow(MeshTopology.Lines, m_NumLinesToDraw * 2, 1);
            resources.textMaterial.SetPass(0);
            Graphics.DrawProceduralNow(MeshTopology.Triangles, m_NumTextBoxesToDraw * 6, 1);
            resources.graphMaterial.SetPass(0);
            Graphics.DrawProceduralNow(MeshTopology.Triangles, m_NumGraphsToDraw * 6, 1);
        }

        int m_NumTextBoxesToDraw = 0;
        int m_NumGraphsToDraw = 0;
        int m_NumLinesToDraw = 0;

        ComputeBuffer m_GraphSampleBuffer; // one big 1D array of floats
        ComputeBuffer m_GraphInstanceBuffer; // one thing for each graph to display
        ComputeBuffer m_TextCellBuffer; // one big 2D array of character cells
        ComputeBuffer m_TextInstanceBuffer; // one thing for each text box to display
        ComputeBuffer m_LineVertexBuffer; // one big 1D array of line vertex positions.

        ComputeBuffer m_ColorBuffer;

        static Managed instance;
        internal static Managed Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new Managed();
                }
                return instance;
            }
            set
            {
                instance = value;
            }
        }

/*
        internal void Render(HDCamera hdCamera, CommandBuffer cmd)
        {
            if (hdCamera.camera.cameraType != CameraType.Game)
                return;
            cmd.DrawProcedural(Matrix4x4.identity, resources.textMaterial, 0, MeshTopology.Triangles, m_NumTextBoxesToDraw * 6, 1);
            cmd.DrawProcedural(Matrix4x4.identity, resources.graphMaterial, 0, MeshTopology.Triangles, m_NumGraphsToDraw * 6, 1);
        }

        internal void Render3D(HDCamera hdCamera, CommandBuffer cmd)
        {
            cmd.DrawProcedural(Matrix4x4.identity, resources.lineMaterial, 0, MeshTopology.Lines, m_NumLinesToDraw, 1);
        }
*/

        public void Dispose()
        {
            m_GraphSampleBuffer?.Dispose();
            m_GraphInstanceBuffer?.Dispose();
            m_TextCellBuffer?.Dispose();
            m_TextInstanceBuffer?.Dispose();
            m_LineVertexBuffer?.Dispose();
            m_ColorBuffer?.Dispose();

            m_GraphSampleBuffer = null;
            m_GraphInstanceBuffer = null;
            m_TextCellBuffer = null;
            m_TextInstanceBuffer = null;
            m_LineVertexBuffer = null;
            m_ColorBuffer = null;

            Unmanaged.Instance.Data.Dispose();
            if (instance == this)
                instance = null;
        }
    }
}
