// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;
using UnityEngine;


public class SolverTrackedTargetType : MonoBehaviour
{
    [SerializeField]
    private SolverHandler solverHandler;

    public void ChangeTrackedTargetTypeHead()
    {
        solverHandler = gameObject.GetComponent<SolverHandler>();

        if (solverHandler != null)
        {
            solverHandler.TrackedTargetType = TrackedObjectType.Head;
        }
    }

    public void ChangeTrackedTargetTypeHandJoint()
    {
        solverHandler = gameObject.GetComponent<SolverHandler>();

        if (solverHandler != null)
        {
            solverHandler.TrackedTargetType = TrackedObjectType.HandJoint;
        }
    }

}

