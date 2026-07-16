// Input-only bridge: Unity XR Hands -> versioned Quest datagrams.
// No calibration, filtering, scaling, trajectory, IK, safety, or robot behavior.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR.Hands;

namespace EmbodiedLab.MotionInput.Quest
{
    public sealed class QuestUmipPublisher : MonoBehaviour
    {
        [SerializeField] private string receiverHost = "192.168.1.2";
        [SerializeField] private int receiverPort = 7060;
        [SerializeField] private string referenceSpace = "local_floor";
        [SerializeField] private bool includeJoints = true;

        private static readonly List<XRHandSubsystem> HandSubsystems = new();
        private static readonly XRHandJointID[] JointIds =
        {
            XRHandJointID.Palm,
            XRHandJointID.Wrist,
            XRHandJointID.ThumbMetacarpal,
            XRHandJointID.ThumbProximal,
            XRHandJointID.ThumbDistal,
            XRHandJointID.ThumbTip,
            XRHandJointID.IndexMetacarpal,
            XRHandJointID.IndexProximal,
            XRHandJointID.IndexIntermediate,
            XRHandJointID.IndexDistal,
            XRHandJointID.IndexTip,
            XRHandJointID.MiddleMetacarpal,
            XRHandJointID.MiddleProximal,
            XRHandJointID.MiddleIntermediate,
            XRHandJointID.MiddleDistal,
            XRHandJointID.MiddleTip,
            XRHandJointID.RingMetacarpal,
            XRHandJointID.RingProximal,
            XRHandJointID.RingIntermediate,
            XRHandJointID.RingDistal,
            XRHandJointID.RingTip,
            XRHandJointID.LittleMetacarpal,
            XRHandJointID.LittleProximal,
            XRHandJointID.LittleIntermediate,
            XRHandJointID.LittleDistal,
            XRHandJointID.LittleTip,
        };

        private UdpClient udp;
        private XRHandSubsystem subsystem;
        private string sessionId;
        private ulong leftSequence;
        private ulong rightSequence;

        private void OnEnable()
        {
            if (receiverPort < 1 || receiverPort > 65535)
                throw new InvalidOperationException("Quest UMIP receiver port must be in [1, 65535].");
            sessionId = Guid.NewGuid().ToString("N");
            udp = new UdpClient();
            udp.Connect(receiverHost, receiverPort);
            SubsystemManager.GetSubsystems(HandSubsystems);
            subsystem = HandSubsystems.Find(candidate => candidate.running);
            if (subsystem == null)
                throw new InvalidOperationException("No running XRHandSubsystem. Enable OpenXR Hand Tracking.");
            subsystem.updatedHands += OnUpdatedHands;
        }

        private void OnDisable()
        {
            if (subsystem != null)
                subsystem.updatedHands -= OnUpdatedHands;
            subsystem = null;
            udp?.Dispose();
            udp = null;
        }

        private void OnUpdatedHands(
            XRHandSubsystem handSubsystem,
            XRHandSubsystem.UpdateSuccessFlags updateFlags,
            XRHandSubsystem.UpdateType updateType)
        {
            // Dynamic is emitted once per frame and is Unity's recommended game-logic update.
            if (updateType != XRHandSubsystem.UpdateType.Dynamic || udp == null)
                return;
            Publish(handSubsystem.leftHand, "left", leftSequence++);
            Publish(handSubsystem.rightHand, "right", rightSequence++);
        }

        private void Publish(XRHand hand, string side, ulong sequence)
        {
            long captureNs = checked((long)(Time.realtimeSinceStartupAsDouble * 1_000_000_000.0));
            var builder = new StringBuilder(8192);
            builder.Append('{');
            Field(builder, "schema", "quest-hand-frame");
            Field(builder, "version", "1.0");
            Field(builder, "session_id", sessionId);
            Field(builder, "stream_id", $"quest/{sessionId}/{side}");
            NumberField(builder, "sequence_number", sequence);
            Field(builder, "side", side);
            Field(builder, "reference_space", referenceSpace);
            Field(builder, "basis", "unity");
            builder.Append("\"capture_timestamp\":{");
            NumberField(builder, "nanoseconds", captureNs);
            Field(builder, "clock_id", $"quest:{sessionId}:unity_realtime", false);
            builder.Append("},");
            builder.Append("\"device_timestamp\":null,");
            Field(builder, "tracking_state", hand.isTracked ? "tracking" : "not_tracking");
            builder.Append("\"tracking_confidence\":null,");
            builder.Append("\"wrist_pose\":");
            if (hand.isTracked) AppendPose(builder, hand.rootPose); else builder.Append("null");
            builder.Append(',');
            builder.Append("\"palm_pose\":");
            if (hand.isTracked && hand.GetJoint(XRHandJointID.Palm).TryGetPose(out Pose palmPose))
                AppendPose(builder, palmPose);
            else
                builder.Append("null");
            builder.Append(',');
            builder.Append("\"articulation\":");
            if (includeJoints && hand.isTracked) AppendArticulation(builder, hand); else builder.Append("null");
            builder.Append(',');
            builder.Append("\"metadata\":{");
            Field(builder, "unity_update_type", "dynamic");
            Field(builder, "confidence_scale", "unavailable", false);
            builder.Append("}}");

            byte[] bytes = Encoding.UTF8.GetBytes(builder.ToString());
            udp.Send(bytes, bytes.Length);
        }

        private static void AppendArticulation(StringBuilder builder, XRHand hand)
        {
            builder.Append("{\"joints\":[");
            bool first = true;
            foreach (XRHandJointID id in JointIds)
            {
                XRHandJoint joint = hand.GetJoint(id);
                if (!joint.TryGetPose(out Pose pose))
                    continue;
                if (!first) builder.Append(',');
                first = false;
                builder.Append('{');
                Field(builder, "name", JointName(id));
                builder.Append("\"pose\":");
                AppendPose(builder, pose);
                builder.Append(',');
                Field(builder, "tracking_state", "tracking");
                builder.Append("\"confidence\":null,");
                if (joint.TryGetRadius(out float radius))
                    builder.AppendFormat(CultureInfo.InvariantCulture, "\"radius_m\":{0:R}", radius);
                else
                    builder.Append("\"radius_m\":null");
                builder.Append('}');
            }
            builder.Append("],\"gestures\":[],\"pinch_strength\":null,");
            builder.Append("\"grasp_strength\":null,\"confidence\":null}");
        }

        private static string JointName(XRHandJointID id)
        {
            // XRHandJointID follows the OpenXR 26-joint semantic layout.
            return id.ToString().ToLowerInvariant();
        }

        private static void AppendPose(StringBuilder builder, Pose pose)
        {
            builder.Append("{\"position_m\":[");
            builder.AppendFormat(CultureInfo.InvariantCulture, "{0:R},{1:R},{2:R}",
                pose.position.x, pose.position.y, pose.position.z);
            builder.Append("],\"orientation_xyzw\":[");
            builder.AppendFormat(CultureInfo.InvariantCulture, "{0:R},{1:R},{2:R},{3:R}",
                pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
            builder.Append("]}");
        }

        private static void Field(StringBuilder builder, string name, string value, bool comma = true)
        {
            builder.Append('"').Append(Escape(name)).Append("\":\"")
                .Append(Escape(value)).Append('"');
            if (comma) builder.Append(',');
        }

        private static void NumberField(StringBuilder builder, string name, long value, bool comma = true)
        {
            builder.Append('"').Append(Escape(name)).Append("\":")
                .Append(value.ToString(CultureInfo.InvariantCulture));
            if (comma) builder.Append(',');
        }

        private static void NumberField(StringBuilder builder, string name, ulong value, bool comma = true)
        {
            builder.Append('"').Append(Escape(name)).Append("\":")
                .Append(value.ToString(CultureInfo.InvariantCulture));
            if (comma) builder.Append(',');
        }

        private static string Escape(string value) => value.Replace("\\", "\\\\").Replace("\"", "\\\"");
    }
}
