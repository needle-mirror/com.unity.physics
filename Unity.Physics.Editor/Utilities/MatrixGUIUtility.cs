using System.Collections.Generic;
using System.Linq;
using UnityEditor;

namespace Unity.Physics.Editor
{
    static class MatrixGUIUtility
    {
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
                        ? "Target has non-uniform scale, which is not supported at run-time."
                        : "One or more targets has non-uniform scale, which is not supported at run-time."
                );
                return MessageType.Warning;
            }

            return MessageType.None;
        }
    }
}
