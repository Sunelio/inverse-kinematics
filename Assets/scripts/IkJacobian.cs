using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IkJacobian : MonoBehaviour
{
    IKManager manager;
    [Header("IK Settings")]
    private float acceptableError = 0.01f;
    private float simulationSpeed = 1.0f;
    private float dampingCoefficient = 0.1f;
    
    private float[] jointAngles;
    private Matrix4x4 jacobian;
    private Vector4 targetGoalVector;
    private Vector3 targetGoal;

    private void Start()
    {
        manager = GetComponent<IKManager>();
        jointAngles = new float[manager.joints.Length * 3];
    }

    public void IKJacobianFunc()
    {
        if (Vector3.Distance(manager.joints[^1].position, manager.target.position) > acceptableError)
        {
            if (manager.rotationAxisConstraint.x != 0) SolveIK(0);
            if (manager.rotationAxisConstraint.y != 0) SolveIK(1);
            if (manager.rotationAxisConstraint.z != 0) SolveIK(2);
        }
    }
    
    private void SolveIK(int axis)
    {
        CalculateJacobianMatrix(axis);
        ApplyJaccobian(axis);
        ApplyJointAngle(axis);
    }

    private void CalculateJacobianMatrix(int axis)
    {
        // on vient chercher le dernier joint de la liste via [^1] que lon soustrais a notre target position ce qui nous donne notre dist target
        targetGoal = manager.target.position - manager.joints[^1].position;
        targetGoalVector = new Vector4(targetGoal.x, targetGoal.y, targetGoal.z,1.0f);
        for (int i = 0; i < manager.joints.Length- 1 ; i++)
        {
            // merci a c# 8.0 qui permet de simplifier en donnant le resulta du switch ici il choit entre le vecteur right up ou forward
            Vector3 rotation = axis switch
            {
                0 => manager.joints[i].right,
                1 => manager.joints[i].up,
                2 => manager.joints[i].forward,
                // par default on retourn le vecteur 0
                _ => Vector3.zero
            };
            Vector3 jointToEnd = manager.joints[^1].position - manager.joints[i].position;
            Vector3 cross = Vector3.Cross(rotation, jointToEnd);
            jacobian.SetColumn(i,new Vector4(cross.x, cross.y, cross.z, 0.0f));
        }
        jacobian.SetColumn(manager.joints.Length - 1, Vector4.zero);
    }

    private void ApplyJaccobian(int axis)
    {
        Matrix4x4 jacobianTranspose = Matrix4x4.Scale(Vector3.one * simulationSpeed) * jacobian.transpose;
        targetGoalVector = jacobianTranspose * targetGoalVector;
    }

    private void ApplyJointAngle(int axis)
    {
        for (int i = 0; i < manager.joints.Length - 1; i++)
        {
            float newAngle = jointAngles[i] + targetGoalVector[i];
            jointAngles[i] = Mathf.Clamp(newAngle, -manager.jointAngleLimit, manager.jointAngleLimit);

            Vector3 eulerAngles = manager.joints[i].localEulerAngles;
            eulerAngles[axis] = jointAngles[i];
            manager.joints[i].localEulerAngles = eulerAngles;
        }
    }

}
