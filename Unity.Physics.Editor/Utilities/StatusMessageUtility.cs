using System.Collections.Generic;
using System.Linq;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;
using UnityObject = UnityEngine.Object;

namespace Unity.Physics.Editor
{
    static class StatusMessageUtility
    {
        public static string GetHierarchyStatusMessage(IReadOnlyList<UnityObject> targets)
        {
            if (targets.Count == 0)
                return string.Empty;

            var numChildTargets = 0;
            foreach (Component c in targets)
            {
                // hierarchy roots and leaf shapes do not emit a message
                if (
                    c == null
                    || c.transform.parent == null
                    || PhysicsShapeExtensions.GetPrimaryBody(c.gameObject) != c.gameObject
                )
                    continue;

                var targetType = c.GetType();
                // only bodies (both explicit and implicit static bodies) will emit a message
                if (
                    targetType == typeof(PhysicsBody)
                    || targetType == typeof(Rigidbody)
                    || c.GetComponent<PhysicsBody>() == null && c.GetComponent<Rigidbody>() == null
                )
                    ++numChildTargets;
            }

            switch (numChildTargets)
            {
                case 0:
                    return string.Empty;
                case 1:
                    return L10n.Tr("Target will be un-parented during the conversion process in order to take part in physics simulation.");
                default:
                    return L10n.Tr("One or more targets will be un-parented during the conversion process in order to take part in physics simulation.");
            }
        }

        public static MessageType GetMatrixStatusMessage(
            IReadOnlyList<MatrixState> matrixStates, out string statusMessage
        )
        {
            statusMessage = string.Empty;
            if (matrixStates.Contains(MatrixState.NotValidTRS))
            {
                statusMessage = L10n.Tr(
                    matrixStates.Count == 1
                        ? "Target's local-to-world matrix is not a valid transformation."
                        : "One or more targets' local-to-world matrices are not valid transformations."
                );
                return MessageType.Error;
            }

            if (matrixStates.Contains(MatrixState.ZeroScale))
            {
                statusMessage =
                    L10n.Tr(matrixStates.Count == 1 ? "Target has zero scale." : "One or more targets has zero scale.");
                return MessageType.Warning;
            }

            if (matrixStates.Contains(MatrixState.NonUniformScale))
            {
                statusMessage = L10n.Tr(
                    matrixStates.Count == 1
                        ? "Target has non-uniform scale. Scale will be baked into the shape data during conversion."
                        : "One or more targets has non-uniform scale. Scale will be baked into the shape data during conversion."
                );
                return MessageType.Warning;
            }

            return MessageType.None;
        }
    }
}
