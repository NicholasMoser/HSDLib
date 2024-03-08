﻿using HSDRaw.AirRide.Gr.Data;
using HSDRawViewer.Rendering;
using HSDRawViewer.Rendering.GX;
using HSDRawViewer.Rendering.Models;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;

namespace HSDRawViewer.Tools
{
    public class SpatialTriangle
    {
        public int Index;

        public Vector3 p1;
        public Vector3 p2;
        public Vector3 p3;

        public Vector3 Min => Vector3.ComponentMin(p1, Vector3.ComponentMin(p2, p3));

        public Vector3 Max => Vector3.ComponentMax(p1, Vector3.ComponentMax(p2, p3));

        public override string ToString()
        {
            return $"{p1} {p2} {p3}";
        }
    }

    public class SpatialBox
    {
        public static readonly int MAX_TRIANGLE_COUNT = 80;

        private BoundingBox box;

        public float MinX => box.Min.X;
        public float MinY => box.Min.Y;
        public float MinZ => box.Min.Z;

        public float MaxX => box.Max.X;
        public float MaxY => box.Max.Y;
        public float MaxZ => box.Max.Z;


        public List<SpatialTriangle> _triangles = new List<SpatialTriangle>();

        public int TriangleCount => _triangles.Count;

        public SpatialBox Child1 { get; internal set; }
        public SpatialBox Child2 { get; internal set; }

        public float Depth { get; internal set; } = 0;

        public SpatialBox(Vector3 min, Vector3 max)
        {
            box = new BoundingBox(min, max);
        }

        public void AddTriangle(SpatialTriangle t)
        {
            if (!box.Intersects(t.p1, t.p2, t.p3))
                return;

            _triangles.Add(t);
        }

        public bool ContainsPoly(IEnumerable<SpatialTriangle> tris)
        {
            foreach (var t in tris)
            {
                if (box.Intersects(t.p1, t.p2, t.p3))
                    return true;
            }

            return false;
        }

        public void Optimize()
        {
            if (TriangleCount > MAX_TRIANGLE_COUNT)
            {
                var min = box.Min;
                var max = box.Max;
                var mid = box.Center;
                var ext = box.Extents;

                if (ext.X > ext.Y && ext.X > ext.Z)
                {
                    Child1 = new SpatialBox(
                        new Vector3(min.X, min.Y, min.Z),
                        new Vector3(mid.X, max.Y, max.Z));
                    Child2 = new SpatialBox(
                        new Vector3(mid.X, min.Y, min.Z),
                        new Vector3(max.X, max.Y, max.Z));
                }
                else if (ext.Y > ext.Z)
                {
                    Child1 = new SpatialBox(
                        new Vector3(min.X, min.Y, min.Z),
                        new Vector3(max.X, mid.Y, max.Z));
                    Child2 = new SpatialBox(
                        new Vector3(min.X, mid.Y, min.Z),
                        new Vector3(max.X, max.Y, max.Z));
                }
                else
                {
                    Child1 = new SpatialBox(
                        new Vector3(min.X, min.Y, min.Z),
                        new Vector3(max.X, max.Y, mid.Z));
                    Child2 = new SpatialBox(
                        new Vector3(min.X, min.Y, mid.Z),
                        new Vector3(max.X, max.Y, max.Z));
                }

                foreach (var t in _triangles)
                {
                    Child1.AddTriangle(t);
                    Child2.AddTriangle(t);
                }

                // clear triangles
                _triangles.Clear();

                // set child depth
                Child1.Depth = Depth + 1;
                Child2.Depth = Depth + 1;

                // optimize new children
                Child1.Optimize();
                Child2.Optimize();
            }
            else
            {
                Console.WriteLine($"Bucket {Depth} {TriangleCount}");
            }
        }

        public override string ToString()
        {
            return $"{box.Min.ToString()} {box.Max.ToString()} {_triangles.Count}";
        }
    }

    public class SpatialPartitionOrganizer
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="triangles"></param>
        /// <returns></returns>
        private static SpatialBox Organize(IEnumerable<SpatialTriangle> triangles)
        {
            Vector3 min = new Vector3(float.MaxValue);
            Vector3 max = new Vector3(float.MinValue);

            foreach (var t in triangles)
            {
                min = Vector3.ComponentMin(min, t.Min);
                max = Vector3.ComponentMax(max, t.Max);
            }

            min = new Vector3(-5000, -5000, -5000);
            max = new Vector3(5000, 5000, 5000);

            var root = new SpatialBox(min, max);
            foreach (var t in triangles)
                root.AddTriangle(t);
            root.Optimize();

            return root;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="model"></param>
        /// <param name="coll"></param>
        /// <returns></returns>
        public static KAR_grCollisionTree GeneratePartition(LiveJObj model, KAR_grCollisionNode coll)
        {
            var _vertices = coll.Vertices;
            var _triangles = coll.Triangles;
            var _joints = coll.Joints;

            // add triangles
            List<SpatialTriangle> triangles = new List<SpatialTriangle>();
            foreach (var j in _joints)
            {
                Matrix4 trans = model.GetJObjAtIndex(j.BoneID).WorldTransform;
                for (int i = j.FaceStart; i < j.FaceStart + j.FaceSize; i++)
                {
                    var tri = _triangles[i];

                    if (tri.SegmentMove)
                        continue;

                    var v1 = GXTranslator.toVector3(_vertices[tri.V3]);
                    var v2 = GXTranslator.toVector3(_vertices[tri.V2]);
                    var v3 = GXTranslator.toVector3(_vertices[tri.V1]);

                    triangles.Add(new SpatialTriangle()
                    {
                        Index = i,
                        p1 = Vector3.TransformPosition(v1, trans),
                        p2 = Vector3.TransformPosition(v2, trans),
                        p3 = Vector3.TransformPosition(v3, trans),
                    });
                }
            }
            // create initial bucket
            var root = Organize(triangles);

            // gather rough lookup
            Dictionary<int, ushort> triangleToRough = new Dictionary<int, ushort>();
            for (int i = 0; i < _triangles.Length; i++)
            {
                var t = _triangles[i];

                if (t.Rough != 0)
                    triangleToRough.Add(i, (ushort)triangleToRough.Count);
            }

            // generate space triangles for zones
            var zvertices = coll.ZoneVertices;
            var ztriangles = coll.ZoneTriangles;
            var zjoints = coll.ZoneJoints;
            List<List<SpatialTriangle>> zonetris = new List<List<SpatialTriangle>>();
            if (zjoints != null)
            foreach (var j in zjoints)
            {
                List<SpatialTriangle> zt = new List<SpatialTriangle>();
                Matrix4 trans = model.GetJObjAtIndex(j.BoneID).WorldTransform;

                for (int i = j.ZoneFaceStart; i < j.ZoneFaceStart + j.ZoneFaceSize; i++)
                {
                    var tri = ztriangles[i];

                    var v1 = GXTranslator.toVector3(zvertices[tri.V3]);
                    var v2 = GXTranslator.toVector3(zvertices[tri.V2]);
                    var v3 = GXTranslator.toVector3(zvertices[tri.V1]);

                    zt.Add(new SpatialTriangle()
                    {
                        p1 = Vector3.TransformPosition(v1, trans),
                        p2 = Vector3.TransformPosition(v2, trans),
                        p3 = Vector3.TransformPosition(v3, trans),
                    });
                }

                zonetris.Add(zt);
            }

            // gather partition data
            List<KAR_grPartitionBucket> partBuckets = new List<KAR_grPartitionBucket>();
            List<ushort> collTris = new List<ushort>();
            List<ushort> roughTris = new List<ushort>();
            List<ushort> zones = new List<ushort>();

            // process spatial buckets
            void processBucket(SpatialBox b)
            {
                // create partition data
                var pt = new KAR_grPartitionBucket()
                {
                    Child1 = -1,
                    Child2 = -1,
                    CollTriangleStart = (ushort)collTris.Count,
                    RoughStart = (ushort)roughTris.Count,
                    ZoneIndexStart = (ushort)zones.Count,
                    MinX = b.MinX,
                    MinY = b.MinY,
                    MinZ = b.MinZ,
                    MaxX = b.MaxX,
                    MaxY = b.MaxY,
                    MaxZ = b.MaxZ,
                    Depth = (byte)b.Depth,
                };
                partBuckets.Add(pt);

                // tris
                foreach (var tri in b._triangles)
                {
                    var t = _triangles[tri.Index];

                    // skip seg move
                    if (t.SegmentMove)
                        continue;

                    // add rough 
                    if (triangleToRough.ContainsKey(tri.Index))
                    {
                        roughTris.Add(triangleToRough[tri.Index]);
                    }

                    // add regardless of rough?
                    collTris.Add((ushort)tri.Index);
                }

                // check zone collisions
                for (int i = 0; i < zonetris.Count; i++)
                {
                    if (b.ContainsPoly(zonetris[i]))
                    {
                        zones.Add((ushort)i);
                    }
                }

                // set counts
                pt.CollTriangleCount = (ushort)(collTris.Count - pt.CollTriangleStart);
                pt.RoughCount = (ushort)(roughTris.Count - pt.RoughStart);
                pt.ZoneIndexCount = (ushort)(zones.Count - pt.ZoneIndexStart);

                // process children
                if (b.Child1 != null && b.Child2 != null)
                {
                    pt.Child1 = (short)(partBuckets.Count);
                    processBucket(b.Child1);

                    pt.Child2 = (short)(partBuckets.Count);
                    processBucket(b.Child2);
                }
            };
            processBucket(root);

            // create partition node
            KAR_grCollisionTree partition = new KAR_grCollisionTree();

            // set buckets
            partition.Buckets = partBuckets.ToArray();

            // set collidable triangles
            partition.CollidableTriangleDataType = 5;
            partition.CollidableTriangles = collTris.ToArray();
            // partition.CollidableTriangleCount = (ushort)partition.CollidableTriangles.Length;

            // set zones
            if (zones.Count > 0)
            {
                partition.ZoneIndexType = 5;
                partition.ZoneIndices = zones.ToArray();
                // partition.ZoneIndexCount = (ushort)partition.ZoneIndices.Length;
            }

            // set rough triangles
            if (roughTris.Count > 0)
            {
                partition.RoughTriangleType = 5;
                partition.RoughIndices = roughTris.ToArray();
                // partition.RoughIndexCount = (ushort)partition.RoughIndices.Length;
            }

            // process bit table
            partition.BitTableDataType = 3;
            partition._s.SetBuffer(0x54, new byte[(int)Math.Ceiling(collTris.Count / 8f)]);
            partition.BitTableCount = (ushort)collTris.Count;

            return partition;
        }
    }
}
