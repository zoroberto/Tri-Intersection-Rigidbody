using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ExporterImporter;
using Octree;
using PBD;
using UnityEngine;
using UnityEngine.Rendering;

public class CPURigid : MonoBehaviour
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
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;

    [HideInInspector]
    private int nodeCount;
    private int triCount; // size of triangle
    private Vector3[] Positions;
    private Vector3[] Velocities;
    List<Triangle> triangles = new List<Triangle>();
    //for render
    private ComputeBuffer vertsBuff = null;
    private ComputeBuffer triBuffer = null;
    private Vector2Int[] indicies;
    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private BoundingBox floorBB;
    BoundingBox[] cusBB;
    private List<PairData> pairIndexL2 = new List<PairData>();
    private readonly int octree_size = 9;

     // collidable pair
    private readonly List<Vector2Int> collidableTriIndex = new List<Vector2Int>();

     [Header("Debug Mode")]
    private OctreeData[] cusOctree;
    private List<int> collidableBoxIndex = new List<int>();
    public bool debugBox = true; 
    public bool boxLv0 = true; 
    public bool boxLv1 = true; 
    public bool debugTri = true; 

    public class Tri
    {
        public Vector3 vertex0, vertex1, vertex2; // �ﰢ���� ������
        public Vector3 p_vertex0, p_vertex1, p_vertex2; // �ﰢ���� ���� ��ġ
        public Vector3 vel0, vel1, vel2;
        public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
        public float deltaTime = 0.01f;
        public void setZeroGravity()
        {
            this.gravity = Vector3.zero;
        }
        public void setInverseGravity()
        {
            this.gravity *= -1.0f;
        }
        public Vector3 getAverageVelocity()
        {
            return (this.vel0 + this.vel1 + this.vel2) / 3.0f;
        }
    }

     public class Line
    {
        public Vector3 p0, p1; // �ﰢ���� ������

        public Vector3 direction
        {
            get { return (p1 - p0).normalized; }
        }

        public Vector3 origin
        {
            get { return p0; }
        }
    }


   
    private List<Tri> posTriangles;
    private List<List<int>> triAABBRelation = new List<List<int>> ();

     struct TriRelation
    {
        public int[] boxSize; 
    }
    private List<TriRelation> triRelation ;
    Vector3 hitPoint = new Vector3();

     void Start()
    {
        
         //obj = gameObject;
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material

        SelectModelName();
        setupMeshData();
        setupShader();
        setBuffData();
        FindFloorMinMax();
        UpdateTriangleRelation();
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

    private void setupMeshData()
    {
        var number = numberOfObjects;
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);
        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        cusOctree = new OctreeData[numberOfObjects * octree_size];
        cusBB = new BoundingBox[numberOfObjects ];

        /// TODO:
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
            vDataArray[i] = new vertData
            {
                pos = Positions[i],
                norms = Vector3.zero,
                uvs = Vector3.zero
            };
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);
        LoadTetModel.ClearData();

        posTriangles = new List<Tri>();

        // print("node count: " + nodeCount);
        // print("tri  count: " + triCount);
        // print("triArray  count: " + triArray.Length);
    }

    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {
        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);
        //Quaternion rotate = new Quaternion(0, 0, 0, 0);
        //transform.Rotate(rotate.eulerAngles);
        //transform.
        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }
    
    private void FindFloorMinMax()
    {
        Vector3[] vertices;

        vertices = floor.GetComponent<MeshFilter>().mesh.vertices;
        floorBB.min = floor.transform.TransformPoint(vertices[0]);
        floorBB.max = floor.transform.TransformPoint(vertices[0]);

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 allVerts = floor.transform.TransformPoint(vertices[i]);

            floorBB.min.x = Mathf.Min(floorBB.min.x, allVerts.x);
            floorBB.min.y = Mathf.Min(floorBB.min.y, allVerts.y);
            floorBB.min.z = Mathf.Min(floorBB.min.z, allVerts.z);

            floorBB.max.x = Mathf.Max(floorBB.max.x, allVerts.x);
            floorBB.max.y = Mathf.Max(floorBB.max.y, allVerts.y);
            floorBB.max.z = Mathf.Max(floorBB.max.z, allVerts.z);
            floorBB.max.y += 0.01f;
        }
    }  


    // find box by inside triagle
    private void UpdateTriangleRelation() {
        // how many box in your system : numberOfObjects * octree_size
        // triAABBRelation = new List<List<int>>();        

        //update bounding box's position(min, max)
        ImplementOctree();        

        triRelation = new List<TriRelation> ();

        // //calculate which triangles in every box
        for(int i = 0; i < triCount; i++) {
            triAABBRelation.Add(new List<int>());
            
            int maxBoxSize = 6 ;
            TriRelation relation = new TriRelation
            {
                boxSize = new int[maxBoxSize] 
            };
            triRelation.Add(relation);
        }


        for(int t = 0; t < triCount; t++) 
        {
            for(int i = 0; i < 6; i++) 
            {
                triRelation[t].boxSize[i] = -1;
            }

            int count = 0;
            for(int i = 0; i< numberOfObjects * octree_size; i++)
            {   
                if(i % octree_size == 0) continue;
                if (
                    CheckTriContainsBox(Positions[triArray[t * 3 + 0]], i) ||
                    CheckTriContainsBox(Positions[triArray[t * 3 + 1]], i) ||
                    CheckTriContainsBox(Positions[triArray[t * 3 + 2]], i) )
                    {
                        // print($"tri {t} {i}");
                        if(!triAABBRelation[t].Contains(i)) triAABBRelation[t].Add(i);
                        
                        triRelation[t].boxSize[count] = i;
                        count++;
                    }
            }

        }
        // for(int i = 0; i < triRelation[1].boxSize.Length; i++) 
        // {
        //     print($"index {i} box {triRelation[1].boxSize[i]}");    
        // }
    }

    private bool CheckTriContainsBox (Vector3 triVec3, int i) 
    {
        Vector3 minPos, maxPos;
        minPos = cusOctree[i].min;
        maxPos = cusOctree[i].max;

        if( triVec3.x >= minPos.x && triVec3.x <= maxPos.x &&
            triVec3.y >= minPos.y && triVec3.y <= maxPos.y &&            
            triVec3.z >= minPos.z && triVec3.z <= maxPos.z)
            return true; 
            
        return false;
    }
   
    void Update()
    {
        UpdateNodes();
        ImplementOctree();
        UpdateReverseVelocity();

        computeVertexNormal();
        vertsBuff.SetData(vDataArray);

        CheckTriIntersection();

        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 100);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);
    }

    void computeVertexNormal()
    {
        posTriangles.Clear();
        for (int i = 0; i < triCount; i++)
        {
            Vector3 v1 = Positions[triArray[i * 3 + 0]];
            Vector3 v2 = Positions[triArray[i * 3 + 1]];
            Vector3 v3 = Positions[triArray[i * 3 + 2]];

            Vector3 N = Vector3.Cross(v2 - v1, v3 - v1);

            vDataArray[triArray[i * 3 + 0]].norms += N;
            vDataArray[triArray[i * 3 + 1]].norms += N;
            vDataArray[triArray[i * 3 + 2]].norms += N;

            posTriangles.Add(new Tri
            {
                vertex0 = v1,
                vertex1 = v2,
                vertex2 = v3
            });            
        }

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i].norms = vDataArray[i].norms.normalized;
        }
    }

    void UpdateNodes()
    {
        //Euler method
        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos = Positions[i];
            Vector3 vel = Velocities[i];

            vel = vel + gravity * invMass * dt;
            pos = pos + vel * dt;

            Positions[i] = pos;
            Velocities[i] = vel;

            vDataArray[i].pos = Positions[i];
        }
    }

    private void UpdateReverseVelocity() 
    {
        for (int i = 0; i < nodeCount; i++)
        {
            for(int j = 0; j < cusBB.Length; j++) {
                floorBB.collide = Intersection.AABB(cusBB[j].min, cusBB[j].max, 
                floorBB.min, floorBB.max); 
                
                var start = indicies[j].x;
                var end = indicies[j].y;
                
                if (floorBB.collide)
                {
                    if(i>= start && i < end ) {
                        // print("index "+ i);
                        Velocities[i] *= -1;
                        Positions[i].y += .1f;
                        vDataArray[i].pos = Positions[i];
                    }
                }
            }
        }
    }

    private void ImplementOctree() 
    {
        
        for(int i = 0; i < numberOfObjects; i++) {
            var _start = indicies[i].x;
            var _end = indicies[i].y;

            Vector3 minPos = Positions[_start]; // 0 26
            Vector3 maxPos = Positions[_start]; // 0 26

            for(var s = _start; s < _end; s++) {
                Vector3 vertex = Positions[s];

                minPos.x = Mathf.Min(minPos.x, vertex.x);
                minPos.y = Mathf.Min(minPos.y, vertex.y);
                minPos.z = Mathf.Min(minPos.z, vertex.z);

                maxPos.x = Mathf.Max(maxPos.x, vertex.x);
                maxPos.y = Mathf.Max(maxPos.y, vertex.y);
                maxPos.z = Mathf.Max(maxPos.z, vertex.z);

                cusBB[i].min = minPos;
                cusBB[i].max = maxPos;
               
                 // Lv0, Initialize Lv0
                cusOctree[i * octree_size].center = (cusBB[i].max + cusBB[i].min) / 2; // center Lv0
                cusOctree[i * octree_size].min = cusBB[i].min; // min value Lv0
                cusOctree[i * octree_size].max = cusBB[i].max; // max value Lv0

                Vector3 centerOct = cusOctree[i * octree_size].center;
                Vector3 sizeOct = cusOctree[i * octree_size].max - cusOctree[i * octree_size].min;

                // Lv2, Split to 8 children [0-7] 
                cusOctree[i * octree_size + 1].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 1].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 1].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 2].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 2].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 2].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 3].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 3].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 3].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 4].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 4].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 4].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 5].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 5].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 5].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 6].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 6].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 6].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 7].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 7].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 7].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 8].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 8].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 8].center.z = centerOct.z + (sizeOct.z / 4);

                
                for (int j = 1; j <= 8; j++)
                {
                    OctreeData oct = cusOctree[i * octree_size + j];
                    cusOctree[i * octree_size + j].min = oct.Minimum(oct.center, sizeOct / 4);
                    cusOctree[i * octree_size + j].max = oct.Maximum(oct.center, sizeOct / 4);
                }
            }
        }
    }

    private void CheckTriIntersection() 
    {
        collidableTriIndex.Clear();
        collidableBoxIndex.Clear();

        for(int i =0; i< triCount;i++)
        {
            for(int j =0; j< triCount;j++)
            {
                int objIndex1 = (int)Mathf.Floor(i/ (triCount/numberOfObjects));
                int objIndex2 = (int)Mathf.Floor(j/ (triCount/numberOfObjects));

                if(objIndex1==objIndex2) continue;
            
                if(objIndex1 >= objIndex2 )
                {
                    // object1 && object2 index 
                    if(Intersection.AABB(cusOctree[objIndex1*9].min,cusOctree[objIndex1*9].max,
                    cusOctree[objIndex2*9].min,cusOctree[objIndex2*9].max) == false)
                    {
                        continue;
                    }
                    else
                    {
                        if(boxLv0)
                        {
                            if (!collidableBoxIndex.Contains(objIndex1*9)) collidableBoxIndex.Add(objIndex1*9);
                            if (!collidableBoxIndex.Contains(objIndex2*9)) collidableBoxIndex.Add(objIndex2*9);
                        }
                    }

                    // for(int k = 0;k<triAABBRelation[i].Count;k++)
                    // {
                    //     for(int l =0;l<triAABBRelation[j].Count;l++)
                    //     {
                    //         // lv0, lv1 => check
                    //         if(Intersection.AABB(cusOctree[triAABBRelation[i][k]].min, cusOctree[triAABBRelation[i][k]].max,
                    //         cusOctree[triAABBRelation[j][l]].min, cusOctree[triAABBRelation[j][l]].max) == false)
                    //         {
                    //             continue;
                    //         }
                    //         else
                    //         {
                    //               // if(boxLv1)
                    //               // {
                    //                        // if (!collidableBoxIndex.Contains(triAABBRelation[i][k])) collidableBoxIndex.Add(triAABBRelation[i][k]);
                    //                        // if (!collidableBoxIndex.Contains(triAABBRelation[j][l])) collidableBoxIndex.Add(triAABBRelation[j][l]);
                    //               // }
                    //         }
                    //     }
                    // }

                    // for(int k = 0;k<triRelation[i].boxSize.Length;k++)
                    // {                        
                    //     for(int l =0;l<triRelation[j].boxSize.Length;l++)
                    //     {
                    //         if(triRelation[i].boxSize[k] != -1 && triRelation[j].boxSize[l] != -1)
                    //         {
                    //             // print($" index {k} {triRelation[1].boxSize[k]}");
                    //              // lv0, lv1 => check
                                
                    //             if(Intersection.AABB(
                    //                 cusOctree[triRelation[i].boxSize[k]].min, 
                    //                 cusOctree[triRelation[i].boxSize[k]].max,
                    //                 cusOctree[triRelation[j].boxSize[l]].min, 
                    //                 cusOctree[triRelation[j].boxSize[l]].max) == false)
                    //             {
                    //                 continue;
                    //             }
                    //             else
                    //             {
                    //                   // if(boxLv1)
                    //                   // {
                    //                            // if (!collidableBoxIndex.Contains(triAABBRelation[i][k])) collidableBoxIndex.Add(triAABBRelation[i][k]);
                    //                            // if (!collidableBoxIndex.Contains(triAABBRelation[j][l])) collidableBoxIndex.Add(triAABBRelation[j][l]);
                    //                    // }
                    //             }
                    //         }
                           
                    //     }
                    // }

                    var t1 = posTriangles[i];
                    var t2 = posTriangles[j];

                    if (Detection(t1, t2))
                    {                
                        if(debugTri)
                        {
                            //print($" ij {i} {j}");
                            Vector2Int pair = new Vector2Int(i, j);
                            if(!collidableTriIndex.Contains(pair)){
                                collidableTriIndex.Add(pair); 
                            } 
                        }   
                    }                   
                }
            }
        }  
    }

    bool Detection(Tri t1, Tri t2)
    {
        var c1 = CheckEdgeCollision(t1.vertex0, t1.vertex1, t2) || 
        CheckEdgeCollision(t1.vertex0, t1.vertex2, t2) || 
        CheckEdgeCollision(t1.vertex1, t1.vertex2, t2);

        var c2 = CheckEdgeCollision(t2.vertex0, t2.vertex1, t1) || 
        CheckEdgeCollision(t2.vertex0, t2.vertex2, t1) || 
        CheckEdgeCollision(t2.vertex1, t2.vertex2, t1);

        return c1 && c2;
    }

     bool CheckEdgeCollision(Vector3 vertex1, Vector3 vertex2, Tri t)
    {
        var edge = new Line();

        edge.p0 = vertex1;
        edge.p1 = vertex2;

        return Intersect(t, edge, ref hitPoint);
    }

    public bool Intersect(Tri triangle, Line ray, ref Vector3 hit)
    {
        // Vectors from p1 to p2/p3 (edges)
        //Find vectors for edges sharing vertex/point p1
        Vector3 e1 = triangle.vertex1 - triangle.vertex0;
        Vector3 e2 = triangle.vertex2 - triangle.vertex0;

        // Calculate determinant
        Vector3 p = Vector3.Cross(ray.direction, e2);

        //Calculate determinat
        float det = Vector3.Dot(e1, p);

        //if determinant is near zero, ray lies in plane of triangle otherwise not
        if (det > -Mathf.Epsilon && det < Mathf.Epsilon)
        {
            var coplanar = IsPointInsideTriangle(ray.p0, triangle);
            var coplanar2 = IsPointInsideTriangle(ray.p1, triangle);

            if (coplanar) hit = ray.p0;
            if (coplanar2) hit = ray.p1;

            return coplanar || coplanar2;
        }
        float invDet = 1.0f / det;

        //calculate distance from p1 to ray origin
        Vector3 t = ray.origin - triangle.vertex0;

        //Calculate u parameter
        float u = Vector3.Dot(t, p) * invDet;

        //Check for ray hit
        if (u < 0 || u > 1) { return false; }

        //Prepare to test v parameter
        Vector3 q = Vector3.Cross(t, e1);

        //Calculate v parameter
        float v = Vector3.Dot(ray.direction, q) * invDet;

        //Check for ray hit
        if (v < 0 || u + v > 1) { return false; }

        // intersection point
        hit = triangle.vertex0 + u * e1 + v * e2;

        if ((Vector3.Dot(e2, q) * invDet) > Mathf.Epsilon)
        {
            //ray does intersect            
            return true;
        }

        // No hit at all
        return false;
    }

    bool IsPointInsideTriangle(Vector3 point, Tri triangle)
    {
         Vector3 normal = Vector3.Cross(triangle.vertex1 - triangle.vertex0, triangle.vertex2 - triangle.vertex0).normalized;

        // ���� �ﰢ�� ��鿡 ����
        Vector3 projectedPoint = ProjectPointOnPlane(point, normal, triangle.vertex0);

        if (Vector3.Distance(projectedPoint, point) > 0.1) return false;

        //Debug.Log(Vector3.Distance(projectedPoint, point));

        // ������ ���� ���� ���� �Ǵ� ����
        Vector3 edge1 = triangle.vertex1 - triangle.vertex0;
        Vector3 vp1 = projectedPoint - triangle.vertex0;
        if (Vector3.Dot(Vector3.Cross(edge1, vp1), normal) < 0) return false;

        Vector3 edge2 = triangle.vertex2 - triangle.vertex1;
        Vector3 vp2 = projectedPoint - triangle.vertex1;
        if (Vector3.Dot(Vector3.Cross(edge2, vp2), normal) < 0) return false;

        Vector3 edge3 = triangle.vertex0 - triangle.vertex2;
        Vector3 vp3 = projectedPoint - triangle.vertex2;
        if (Vector3.Dot(Vector3.Cross(edge3, vp3), normal) < 0) return false;

        return true; // ��� �˻縦 ����ߴٸ�, ������ ���� �ﰢ�� ���ο� �ֽ��ϴ�.
    }

    Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
    {
        float d = Vector3.Dot(planeNormal, (point - planePoint)) / planeNormal.magnitude;
        return point - d * planeNormal;
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

    private void OnDrawGizmos()
    {
        if (cusOctree != null  )
        {
            for (int i = 0; i < collidableBoxIndex.Count; i++)
                {                    

                    if (boxLv0 && collidableBoxIndex[i] % octree_size == 0)
                    {
                        Gizmos.color = Color.red;
                        
                        Gizmos.DrawWireCube(cusOctree[collidableBoxIndex[i]].center, cusOctree[collidableBoxIndex[i]].max - cusOctree[collidableBoxIndex[i]].min);
                    }

                    if (boxLv1 && collidableBoxIndex[i] % octree_size != 0)
                    {
                        
                        // print($"index {collidableBoxIndex[i]}");
                        Gizmos.color = Color.green;
                        Gizmos.DrawWireCube(cusOctree[collidableBoxIndex[i]].center, cusOctree[collidableBoxIndex[i]].max - cusOctree[collidableBoxIndex[i]].min);
                    }
                }
        }

        if (collidableTriIndex != null && debugTri)
        {
            
            List<int> newList = new List<int>();
            for(int i =0; i< collidableTriIndex.Count; i++){
                if(!newList.Contains(collidableTriIndex[i].x)) newList.Add(collidableTriIndex[i].x);
                if(!newList.Contains(collidableTriIndex[i].y)) newList.Add(collidableTriIndex[i].y);
            }
            for(int i =0; i< newList.Count; i++)
            {
                // DrawTriangle(posTriangles[newList[i]], Color.green);
                DrawTriangle(Positions[triArray[newList[i] * 3 + 0]], Positions[triArray[newList[i] * 3 + 1]], Positions[triArray[newList[i] * 3 + 2]], Color.blue);
            }
        }
    //     if(Positions != null)
    //    DrawTriangle(Positions[triArray[0 * 3 + 0]], Positions[triArray[0 * 3 + 1]], Positions[triArray[0 * 3 + 2]], Color.blue);

    //    if(triAABBRelation != null)
    //    {
    //         Gizmos.color = Color.red;
    //     for(int i = 0; i < triAABBRelation[94].Count; i++) {
    //         if(triAABBRelation[94][i] % octree_size != 0)
    //         Gizmos.DrawWireCube(cusOctree[triAABBRelation[94][i]].center, cusOctree[triAABBRelation[94][i]].max - cusOctree[triAABBRelation[94][i]].min);
    //     }

    //     for(int i = 0; i < triAABBRelation[4].Count; i++) {
    //         if(triAABBRelation[10][i] % octree_size != 0)
    //         Gizmos.DrawWireCube(cusOctree[triAABBRelation[4][i]].center, cusOctree[triAABBRelation[4][i]].max - cusOctree[triAABBRelation[4][i]].min);
    //     }
    //    }

       if(Positions != null && triArray !=null)
       {
            for(int t = 0; t < triCount; t++) {
            
            // DrawTriangle(Positions[triArray[10 * 3 + 0]], Positions[triArray[10 * 3 + 1]], Positions[triArray[10 * 3 + 2]], Color.blue);
            // DrawTriangle(Positions[triArray[1 * 3 + 0]], Positions[triArray[1 * 3 + 1]], Positions[triArray[1 * 3 + 2]], Color.blue);
            }
            // DrawTriangle(Positions[triArray[0 * 3 + 0]], Positions[triArray[0 * 3 + 1]], Positions[triArray[0 * 3 + 2]], Color.blue);
            // DrawTriangle(Positions[triArray[320 * 3 + 0]], Positions[triArray[320 * 3 + 1]], Positions[triArray[320 * 3 + 2]], Color.blue);
            
            // DrawTriangle(Positions[triArray[94 * 3 + 0]], Positions[triArray[94 * 3 + 1]], Positions[triArray[94 * 3 + 2]], Color.blue);
            // DrawTriangle(Positions[triArray[4 * 3 + 0]], Positions[triArray[4 * 3 + 1]], Positions[triArray[4 * 3 + 2]], Color.blue);
            // DrawTriangle(Positions[triArray[10 * 3 + 0]], Positions[triArray[10 * 3 + 1]], Positions[triArray[10 * 3 + 2]], Color.blue);
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
        }
    }
}
