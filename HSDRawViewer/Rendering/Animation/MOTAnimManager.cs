using HSDRaw.Common;
using HSDRaw.Common.Animation;
using HSDRaw.Tools;
using HSDRawViewer.Converters;
using HSDRawViewer.Rendering.Models;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace HSDRawViewer.Rendering
{
    public class MotAnimManager : JointAnimManager
    {
        private MOT_FILE _motFile;
        private short[] _motJointTable;

        public override void ApplyAnimation(LiveJObj jobj, float frame)
        {
            foreach (var v in jobj.Enumerate)
                ApplyAnimation0(v, frame);
        }

        private void ApplyAnimation0(LiveJObj jobj, float frame)
        {
            var joints = _motFile.Joints.FindAll(
                e => e.BoneID >= 0 &&
                e.BoneID < _motJointTable.Length &&
                _motJointTable[e.BoneID] == jobj.Index);

            foreach (var j in joints)
            {
                var key = j.GetKey(frame / 60f);
                if (key == null)
                    continue;

                if (j.TrackFlag.HasFlag(MOT_FLAGS.TRANSLATE))
                    jobj.Translation = new Vector3(jobj.Desc.TX, jobj.Desc.TY, jobj.Desc.TZ) + new Vector3(key.X, key.Y, key.Z);

                if (j.TrackFlag.HasFlag(MOT_FLAGS.SCALE))
                    jobj.Scale = new Vector3(jobj.Desc.SX, jobj.Desc.SY, jobj.Desc.SZ) + new Vector3(key.X, key.Y, key.Z);

                if (j.TrackFlag.HasFlag(MOT_FLAGS.ROTATE))
                    jobj.Rotation.Xyz = MotRotationToEuler(new Vector3(jobj.Desc.RX, jobj.Desc.RY, jobj.Desc.RZ), key);
            }
        }

        /// <summary>
        /// Shared by preview and bake so they can't drift apart.
        /// </summary>
        private static Vector3 MotRotationToEuler(Vector3 bindRot, MOT_KEY key)
        {
            var q = Math3D.EulerToQuat(bindRot.X, bindRot.Y, bindRot.Z);

            var dir = new Vector3(key.X, key.Y, key.Z);
            if (dir.LengthSquared > 1e-12f)
            {
                dir.Normalize();
                float dot = Math.Max(-1f, Math.Min(1f, Vector3.Dot(Vector3.UnitX, dir)));
                float a = (float)Math.Acos(dot);
                var axis = Vector3.Cross(Vector3.UnitX, dir);
                if (Math.Abs(a) > 1e-6f && axis.LengthSquared > 1e-12f)
                    q *= Quaternion.FromAxisAngle(axis.Normalized(), a);
            }

            q *= Quaternion.FromEulerAngles(key.W * (float)Math.PI / 180f, 0, 0);
            return Matrix4.CreateFromQuaternion(q).ExtractRotationEuler();
        }

        public void SetMOT(short[] jointTable, MOT_FILE file)
        {
            _motFile = file;
            _motJointTable = jointTable;
            FrameCount = (int)Math.Ceiling(_motFile.EndTime * 60);
        }

        /// <summary>
        /// Converts the MOT data into per-joint FOBJ tracks so the track editor can use it.
        /// Bakes one key per frame; use "Compress All Tracks" afterwards.
        /// </summary>
        public void BakeToTracks(HSD_JOBJ root)
        {
            var joints = root.TreeList;

            Nodes.Clear();
            for (int i = 0; i < joints.Count; i++)
                Nodes.Add(new AnimNode());

            foreach (var j in _motFile.Joints)
            {
                if (j.Keys.Count == 0 || j.BoneID < 0 || j.BoneID >= _motJointTable.Length)
                    continue;

                int idx = _motJointTable[j.BoneID];
                if (idx < 0 || idx >= joints.Count)
                    continue;

                var node = Nodes[idx];
                var bind = joints[idx];
                Vector3 prev = default;

                for (int f = 0; f <= FrameCount; f++)
                {
                    var key = j.GetKey(f / 60f);
                    if (key == null)
                        continue;

                    if (j.TrackFlag.HasFlag(MOT_FLAGS.TRANSLATE))
                    {
                        node.AddLinearKey(JointTrackType.HSD_A_J_TRAX, f, bind.TX + key.X);
                        node.AddLinearKey(JointTrackType.HSD_A_J_TRAY, f, bind.TY + key.Y);
                        node.AddLinearKey(JointTrackType.HSD_A_J_TRAZ, f, bind.TZ + key.Z);
                    }

                    if (j.TrackFlag.HasFlag(MOT_FLAGS.SCALE))
                    {
                        node.AddLinearKey(JointTrackType.HSD_A_J_SCAX, f, bind.SX + key.X);
                        node.AddLinearKey(JointTrackType.HSD_A_J_SCAY, f, bind.SY + key.Y);
                        node.AddLinearKey(JointTrackType.HSD_A_J_SCAZ, f, bind.SZ + key.Z);
                    }

                    if (j.TrackFlag.HasFlag(MOT_FLAGS.ROTATE))
                    {
                        var e = MotRotationToEuler(new Vector3(bind.RX, bind.RY, bind.RZ), key);
                        if (f > 0)
                        {
                            e.X = Unwrap(e.X, prev.X);
                            e.Y = Unwrap(e.Y, prev.Y);
                            e.Z = Unwrap(e.Z, prev.Z);
                        }
                        prev = e;
                        node.AddLinearKey(JointTrackType.HSD_A_J_ROTX, f, e.X);
                        node.AddLinearKey(JointTrackType.HSD_A_J_ROTY, f, e.Y);
                        node.AddLinearKey(JointTrackType.HSD_A_J_ROTZ, f, e.Z);
                    }
                }
            }
        }

        private static float Unwrap(float v, float prev)
        {
            const float TwoPi = (float)(Math.PI * 2);
            while (v - prev > Math.PI) v -= TwoPi;
            while (v - prev < -Math.PI) v += TwoPi;
            return v;
        }

        /// <summary>
        /// Trims frames to given region
        /// </summary>
        /// <param name="startFrame"></param>
        /// <param name="endFrame"></param>
        public override void Trim(int startFrame, int endFrame)
        {
            FrameCount = endFrame - startFrame;
            _motFile.EndTime = 0f;
            foreach (var j in _motFile.Joints)
            {
                var startKey = j.GetKey(startFrame / 60f);
                var endKey = j.GetKey(endFrame / 60f);
                var middleKeys = j.Keys.FindAll(k => k.Time >= startFrame / 60f && k.Time <= endFrame / 60f);

                j.Keys = new List<MOT_KEY>();
                if (middleKeys.Count == 0)
                {
                    j.Keys.Add(startKey);
                    j.Keys.Add(endKey);
                } else
                {
                    if (middleKeys[0].Time - startKey.Time > 0.001)
                    {
                        j.Keys.Add(startKey);
                    }

                    j.Keys.AddRange(middleKeys);

                    if (endKey.Time - middleKeys[middleKeys.Count - 1].Time > 0.001)
                    {
                        j.Keys.Add(endKey);
                    }
                }

                Debug.Assert(j.Keys.Count > 0);
                foreach (var k in j.Keys)
                {
                    k.Time -= startFrame / 60f;
                }
                j.MaxTime = j.Keys[j.Keys.Count - 1].Time;
                _motFile.EndTime = Math.Max(_motFile.EndTime, j.MaxTime);
            }
        }

        public MOT_FILE GetMOT()
        {
            return _motFile;
        }

    }
}
