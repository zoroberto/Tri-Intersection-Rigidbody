using System.Collections;
using System.Collections.Generic;
using ExporterImporter;
using Octree;
using PBD;
using UnityEngine;
using UnityEngine.Rendering;

public class GPURigid : MonoBehaviour
{
    public enum MyModel
    {
        Cube,
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };

    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public int numberOfObjects = 1;
    public float invMass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    [Header("Import CSV")]
    public string csv_file = "object_positions.csv";

    [Header("Collision")]
    public GameObject floor;

    [Header("Rendering Paramenter")]
    public ComputeShader computeShader;
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;
     private ComputeShader computeShaderObj;

    [HideInInspector]
    private int nodeCount;
    private int triCount; // size of triangle
    
    //for render
    private ComputeBuffer vertsBuff = null;
    private ComputeBuffer triBuffer = null;
   //for compute shader
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer velocitiesBuffer ;
    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;
    private ComputeBuffer posTrianglesBuffer;
    private ComputeBuffer floorBBBuffer ;
    private ComputeBuffer floorPositionsBuffer;
    private ComputeBuffer bbMinMaxBuffer ;
    private ComputeBuffer objectIndexBuffer;
    private ComputeBuffer floorCollisionResultBuffer ;
    private ComputeBuffer bbOctreeBuffer;
    private ComputeBuffer collisionBoxBuffer;
    private ComputeBuffer collisionTriBuffer;
    private ComputeBuffer triAABBRelationBuffer;
    private RenderTexture collisionResultsTexture ; 
    private ComputeBuffer collisionPairsBuffer ; 

    List<Triangle> triangles = new List<Triangle>();
    private int computeVerticesNormal; // for rendering purpose 
    private int FindTriAABBRelationKernel;
    private int FindPosTrianglesKernel;
    private int updatePosKernel;
    private int findFloorMinMaxKernel;
    private int findBBMinMaxKernel; 
    private int updateReverseVelocityKernel;
    private int implementOctreeKernel;
    private int removeCollisionResultsKernel;
    private int TriIntersectionKernel;
    private int RemoveTriKernel;
    private int CheckBoundingBoxCollisionsKernel;

    private Vector3[] Positions;
    private Vector3[] Velocities;
    private Vector2Int[] indicies;
    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    // octree
    private OctreeData[] bbOctree;
    private readonly int octree_size = 9;
    private int[] collisionBox;
    private Vector2Int[] collisionTri;
      private struct Tri
    {
        public Vector3 vertex0, vertex1, vertex2;
    }
    private Tri[] posTriangles;
    private int[] triAABBBoxes;


    struct TriAABBRelation
    {
        public int[] boxSize;
    }

    private List<TriAABBRelation> triRelation ;

    [Header("Debug Mode")]
    public bool debugBox = true;
    public bool boxLv0 = true; 
    public bool boxLv1 = true; 
    public bool debugTri = true; 

    void Start()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        computeShaderObj = Instantiate(computeShader); // to instantiate the compute shader to be use with multiple object

        SelectModelName();
        SetupMeshData();
        SetupShader();
        SetBuffData();
        SetupComputeBuffer();
        SetupKernel();
        SetupComputeShader();
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.Cube: modelName = "33cube.1"; break;
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }
    

    private void SetupMeshData()
    {
        var number = numberOfObjects;
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);
        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        // if(number > csvData.Count) // exit app
        // {

        //     throw new Exception("cc"); 
        //     //  UnityEditor.EditorApplication.isPlaying = false;
            
        // } 
      
        indicies = new Vector2Int[number];
        int st_index = 0;

        var _Positions = LoadTetModel.positions.ToArray();            
        var _triangles = LoadTetModel.triangles;
        var _triArray = LoadTetModel.triangleArr.ToArray();

        Positions = new Vector3[number * LoadTetModel.positions.Count];
        Velocities = new Vector3[number * LoadTetModel.positions.Count];
        triangles = new List<Triangle>(new Triangle[number * LoadTetModel.triangles.Count]);
        triArray = new int[number * LoadTetModel.triangleArr.Count];

         for(int i=0;i<number;i++){
            int PosOffset = i * LoadTetModel.positions.Count;
            
            List<string> column = csvData[i];
            float x = float.Parse(column[0]);
            float y = float.Parse(column[1]);
            float z = float.Parse(column[2]);
            Vector3 Offset = new Vector3(x, y, z);
            
            for(int j=0;j<LoadTetModel.positions.Count;j++){                
                Positions[j+PosOffset] = _Positions[j] + Offset;
            }

            int TriOffset = i * LoadTetModel.triangles.Count;
            for(int j=0;j<LoadTetModel.triangles.Count;j++){
                var t = _triangles[j];
                triangles[j+TriOffset] = new Triangle(t.vertices[0] + PosOffset, t.vertices[1] + PosOffset, t.vertices[2] + PosOffset);
            }

            int TriArrOffset = i * LoadTetModel.triangleArr.Count;
            for(int j=0;j<LoadTetModel.triangleArr.Count;j++){
                triArray[j+TriArrOffset] = _triArray[j] + PosOffset;
            }

            indicies[i] = new Vector2Int(st_index, st_index + LoadTetModel.positions.Count);
            st_index += LoadTetModel.positions.Count;
        }        

        nodeCount = Positions.Length;
        triCount = triangles.Count; //
        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);
        LoadTetModel.ClearData();

        // print("node count: " + nodeCount);
        print("tri  count: " + triCount);
        // print("triArray  count: " + triArray.Length);
    }

     private void SetupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }

     private void SetBuffData()
    {
        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }

    private void SetupComputeBuffer()
    {
        bbOctree = new OctreeData[numberOfObjects * octree_size];
        collisionBox = new int[numberOfObjects * octree_size];
        posTriangles = new Tri[triCount];
        collisionTri = new Vector2Int[triCount];

        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
     
        Dictionary<int, List<int>> nodeTriangles = new Dictionary<int, List<int>>();
        for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
        {
            Triangle tri = triangles[triIndex];
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                int vertex = tri.vertices[vertexIndex];
                if (!nodeTriangles.ContainsKey(vertex))
                {
                    nodeTriangles[vertex] = new List<int>();
                }
                nodeTriangles[vertex].Add(triIndex);
            }
        }
        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            if (nodeTriangles.TryGetValue(i, out List<int> triangleIndexes))
            {
                foreach (int triIndex in triangleIndexes)
                {
                    Triangle tri = triangles[triIndex];
                    MTriangle tmpTri = new MTriangle { v0 = tri.vertices[0], v1 = tri.vertices[1], v2 = tri.vertices[2] };
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());

        Vector3[] _floorVertices = floor.GetComponent<MeshFilter>().mesh.vertices;
        List<Vector3> floorVertices = new List<Vector3>();

        for (int i = 0; i < _floorVertices.Length; i++)
        {
            floorVertices.Add(floor.transform.TransformPoint(_floorVertices[i]));
        }

        floorBBBuffer = new ComputeBuffer(1, sizeof(float) * 6);
        floorPositionsBuffer = new ComputeBuffer(_floorVertices.Length, sizeof(float) * 3);
        floorPositionsBuffer.SetData(floorVertices);

        bbMinMaxBuffer = new ComputeBuffer(numberOfObjects, sizeof(float) * 6);
        objectIndexBuffer = new ComputeBuffer(numberOfObjects, sizeof(int) * 2);
        objectIndexBuffer.SetData(indicies);
        floorCollisionResultBuffer = new ComputeBuffer(numberOfObjects, sizeof(int));
        bbOctreeBuffer = new ComputeBuffer(numberOfObjects * octree_size, sizeof(float) * 9);
        posTrianglesBuffer = new ComputeBuffer(triCount, sizeof(float) * 9);
        collisionBoxBuffer = new ComputeBuffer(numberOfObjects * octree_size, sizeof(int));
        collisionTriBuffer = new ComputeBuffer(triCount, sizeof(int) * 2);

        int maximumBoxSize = 6;
        triAABBRelationBuffer = new ComputeBuffer(triCount, sizeof(int) * maximumBoxSize);

        collisionResultsTexture = new RenderTexture(octree_size* numberOfObjects, octree_size* numberOfObjects, 0);
        collisionResultsTexture.enableRandomWrite = true;
        collisionResultsTexture.Create();
        // change to int2
        // collisionPairsBuffer = new ComputeBuffer();
        
        collisionBox.Initialize();
        collisionTri.Initialize();
        collisionBoxBuffer.SetData(collisionBox);
        collisionTriBuffer.SetData(collisionTri);
    }

    private void SetupKernel()
    {
        FindTriAABBRelationKernel = computeShaderObj.FindKernel("FindTriAABBRelation");
        //for rendering
        computeVerticesNormal = computeShaderObj.FindKernel("computeVerticesNormal");
        FindPosTrianglesKernel = computeShaderObj.FindKernel("FindPosTriangles");
        updatePosKernel = computeShaderObj.FindKernel("UpdatePosKernel");
        findFloorMinMaxKernel = computeShaderObj.FindKernel("FindFloorMinMax");
        findBBMinMaxKernel = computeShader.FindKernel("FindBBMinMax");
        updateReverseVelocityKernel = computeShaderObj.FindKernel("UpdateReverseVelocity");
        implementOctreeKernel = computeShader.FindKernel("ImplementOctree");
        removeCollisionResultsKernel = computeShaderObj.FindKernel("RemoveCollisionResults");
        CheckBoundingBoxCollisionsKernel = computeShaderObj.FindKernel("CheckBoundingBoxCollisions");
        
        
        TriIntersectionKernel = computeShaderObj.FindKernel("TriIntersection");
        RemoveTriKernel = computeShaderObj.FindKernel("RemoveTriKernel");
    }

    private void SetupComputeShader()
    {
        //send uniform data for kernels in compute shader
        computeShaderObj.SetFloat("dt", dt);
        computeShaderObj.SetBool("debugBox", debugBox);
        computeShaderObj.SetBool("debugTri", debugTri);
        computeShaderObj.SetFloat("invMass", invMass);
        computeShaderObj.SetInt("triCount", triCount);
        computeShaderObj.SetInt("nodeCount", nodeCount);
        computeShaderObj.SetInt("numberObj", numberOfObjects);
        computeShaderObj.SetInt("octree_size", octree_size);

        // FindTriAABBRelation
        computeShaderObj.SetBuffer(FindTriAABBRelationKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(FindTriAABBRelationKernel, "triArray", triBuffer);
        computeShaderObj.SetBuffer(FindTriAABBRelationKernel, "bbOctree", bbOctreeBuffer);
        computeShaderObj.SetBuffer(FindTriAABBRelationKernel, "triAABBRelation", triAABBRelationBuffer);

        //computeVerticesNormal
        computeShaderObj.SetBuffer(computeVerticesNormal, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "Triangles", triangleBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "TrianglePtr", triangleIndicesBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "vertsBuff", vertsBuff); //passing to rendering

        // FindPosTrianglesKernel
        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "posTriangles", posTrianglesBuffer);
        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "triArray", triBuffer);

        // UpdatePosKernel
        computeShaderObj.SetBuffer(updatePosKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "vertsBuff", vertsBuff); //passing to rendering

        // FindFloorMinMaxKernel
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorPositions", floorPositionsBuffer);
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorBB", floorBBBuffer);
        computeShaderObj.Dispatch(findFloorMinMaxKernel, 1, 1, 1);

        // findBBMinMaxKernel
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "bbMinMax", bbMinMaxBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "ObjectIndex", objectIndexBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "floorBB", floorBBBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "floorCollisionResults", floorCollisionResultBuffer);
   
        // UpdateReverseVelocity
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "floorCollisionResults", floorCollisionResultBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "vertsBuff", vertsBuff); //passing to rendering
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "ObjectIndex", objectIndexBuffer);

        // ImplementOctree
        computeShaderObj.SetBuffer(implementOctreeKernel, "bbMinMax", bbMinMaxBuffer);
        computeShaderObj.SetBuffer(implementOctreeKernel, "bbOctree", bbOctreeBuffer);

        // removeCollisionResults
        computeShaderObj.SetBuffer(removeCollisionResultsKernel, "collisionBox", collisionBoxBuffer);

        // RemoveTriKernel
        computeShaderObj.SetBuffer(RemoveTriKernel, "collisionTri", collisionTriBuffer);
        
        // TriIntersection
        computeShaderObj.SetBuffer(TriIntersectionKernel, "posTriangles", posTrianglesBuffer); 
        computeShaderObj.SetBuffer(TriIntersectionKernel, "collisionBox", collisionBoxBuffer);
        computeShaderObj.SetBuffer(TriIntersectionKernel, "bbOctree", bbOctreeBuffer);
        computeShaderObj.SetBuffer(TriIntersectionKernel, "collisionTri", collisionTriBuffer); 
        computeShaderObj.SetTexture(TriIntersectionKernel, "collisionResults", collisionResultsTexture);
        computeShaderObj.SetBuffer(TriIntersectionKernel, "triAABBRelation", triAABBRelationBuffer);

        // CheckBoundingBoxCollisions
        computeShaderObj.SetBuffer(CheckBoundingBoxCollisionsKernel, "bbOctree", bbOctreeBuffer);
        computeShaderObj.SetTexture(CheckBoundingBoxCollisionsKernel, "collisionResults", collisionResultsTexture);

        // int[] triAABBBoxes = new int[triCount *6];
        // triAABBBoxes.Initialize();

        // triAABBRelationBuffer.SetData(triAABBBoxes);

        computeShaderObj.Dispatch(implementOctreeKernel, Mathf.CeilToInt(numberOfObjects / 1024f), 1, 1);
        computeShaderObj.Dispatch(FindTriAABBRelationKernel, Mathf.CeilToInt(triCount / 1024.0f), 1, 1);

        // triAABBRelationBuffer.GetData(triAABBBoxes);

        // print($"box indices {triAABBBoxes.Length}");    
        // for(int i = 0; i < 6; i++) 
        // {
        //     // print($"box indices {i}  {triAABBBoxes[i]}");    
        //     // print($"tri {i} index {triAABBBoxes[i]}");
        //     print($"gpu tri { Mathf.Floor(i / 6)} box {i} index {triAABBBoxes[i]}");

        // }
    }

    private bool CheckTriContainsBox(Vector3 triVec3, int i)
    {

        Vector3 minPos, maxPos;
        minPos = bbOctree[i].min;
        maxPos = bbOctree[i].max;

        if( triVec3.x >= minPos.x && triVec3.x <= maxPos.x &&
            triVec3.y >= minPos.y && triVec3.y <= maxPos.y &&            
            triVec3.z >= minPos.z && triVec3.z <= maxPos.z)
            return true; 
            
        return false;
    }


    void Update()
    {
        DispatchComputeShader();
        RenderObject();
        GetDataToCPU();
    }

    void DispatchComputeShader()
    {
        //compute normal for rendering
        computeShaderObj.Dispatch(computeVerticesNormal, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        computeShaderObj.Dispatch(FindPosTrianglesKernel, Mathf.CeilToInt(triCount / 1024.0f), 1, 1);
        
        // compute shader
        computeShaderObj.Dispatch(updatePosKernel, Mathf.CeilToInt(nodeCount / 1024.0f), 1, 1);
        computeShaderObj.Dispatch(findBBMinMaxKernel, Mathf.CeilToInt(numberOfObjects / 1024f), 1, 1);
        computeShaderObj.Dispatch(updateReverseVelocityKernel, Mathf.CeilToInt(nodeCount / 1024f), 1, 1);
        computeShaderObj.Dispatch(implementOctreeKernel, Mathf.CeilToInt(numberOfObjects / 1024f), 1, 1);
        computeShaderObj.Dispatch(removeCollisionResultsKernel, Mathf.CeilToInt(numberOfObjects * octree_size / 1024f), 1, 1);
        computeShaderObj.Dispatch(RemoveTriKernel, Mathf.CeilToInt(triCount / 1024.0f), 1, 1);
        
        int numGroups_BB =  Mathf.CeilToInt(numberOfObjects / 32f);
        computeShaderObj.Dispatch(CheckBoundingBoxCollisionsKernel, numGroups_BB, numGroups_BB, 1);   

        int numGroups_Tri =  Mathf.CeilToInt(triCount / 32f);
        computeShaderObj.Dispatch(TriIntersectionKernel, numGroups_Tri, numGroups_Tri, 1);   
    }

    void RenderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);
    }

    private void GetDataToCPU()
    {
        if (debugBox)
        {
            bbOctreeBuffer.GetData(bbOctree);
            collisionBoxBuffer.GetData(collisionBox);

            // for(int i = 0; i < collisionBoxResults.Length; i++)
            // {
            //     if(collisionBoxResults[i] == 1) {
            //         if (i % octree_size == 0)
            //         print($"res {i} {collisionBoxResults[i]}");
            //     }
            // }
        }



        if(debugTri)
        {
            collisionTriBuffer.GetData(collisionTri);
            posTrianglesBuffer.GetData(posTriangles);
        }

        // triRelation = new List<TriAABBRelation> ();
        // triRelation.Clear();
        //  for(int i = 0; i < triCount; i++) {
        //     TriAABBRelation relation = new TriAABBRelation
        //     {
        //         boxSize = new int[6] 
        //     };
        //     triRelation.Add(relation);
        // }


        //  for(int t = 0; t < triCount; t++) 
        // {
        //     for(int i = 0; i < 6; i++) 
        //     {
        //         triRelation[t].boxSize[i] = -1;
        //     }

        //     int count = 0;
        //     for(int i = 0; i< numberOfObjects * octree_size; i++)
        //     {   
        //         if(i % octree_size == 0) continue;
        //         if (
        //             CheckTriContainsBox(Positions[triArray[t * 3 + 0]], i) ||
        //             CheckTriContainsBox(Positions[triArray[t * 3 + 1]], i) ||
        //             CheckTriContainsBox(Positions[triArray[t * 3 + 2]], i) )
        //             {                        
        //                 triRelation[t].boxSize[count] = i;
        //                 count++;
        //             }
        //     }

        // //     for(int i = 0; i < triRelation[0].boxSize.Length; i++) 
        // //  {

        // //     print($"cpu tri {0} box {i} index {triRelation[0].boxSize[i]}");
        // //  }
        // }
    }

     private void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(20, 40, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 50;
        style.normal.textColor = Color.yellow;

        string text = string.Format("num. Obj :: " + numberOfObjects);
        GUI.Label(rect, text, style);
    }

    void OnDrawGizmos()
    {
        if(collisionBox != null && debugBox)
        {
            for (int i = 0; i < collisionBox.Length; i++)
            {
                if(collisionBox[i] == 1)
                {
                    if(boxLv0 && i % octree_size == 0)
                    {
                        Gizmos.color = Color.red;
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }

                    if(boxLv1 && i % octree_size !!= 0)
                    {
                        Gizmos.color = Color.green;
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }
                }
            }
        }

        if(posTriangles != null && debugTri)
        {
            for (int c = 0; c < collisionTri.Length; c++)
            {
                if (collisionTri[c].x == 1)  DrawTriangle(posTriangles[c], Color.blue);                
            }

            for (int c = 0; c < collisionTri.Length; c++)
            {
                if (collisionTri[c].y == 1)  DrawTriangle(posTriangles[c], Color.blue);
            }
        }
    }

    

    private void DrawTriangle(Tri triangle, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(triangle.vertex0, triangle.vertex1);
        Gizmos.DrawLine(triangle.vertex1, triangle.vertex2);
        Gizmos.DrawLine(triangle.vertex2, triangle.vertex0);
    }

    private void DrawTriangle(Vector3 vertex0, Vector3 vertex1, Vector3 vertex2, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(vertex0, vertex1);
        Gizmos.DrawLine(vertex1, vertex2);
        Gizmos.DrawLine(vertex2, vertex0);
    }

    private void OnDestroy() 
    {
        if (enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
            posTrianglesBuffer.Dispose();
            positionsBuffer.Dispose();
            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();
            velocitiesBuffer.Dispose();
            floorBBBuffer.Dispose();
            floorPositionsBuffer.Dispose();
            bbMinMaxBuffer.Dispose();
            objectIndexBuffer.Dispose();
            floorCollisionResultBuffer.Dispose();
            bbOctreeBuffer.Dispose();
            collisionBoxBuffer.Dispose();
            collisionTriBuffer.Dispose();
        }    
    }
}