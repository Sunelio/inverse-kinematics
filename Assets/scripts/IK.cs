using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;


 public enum IKTypeCalc
{
    JACOBIAN,

}

public class IK : MonoBehaviour
{
    public List<Transform> joints = new List<Transform>();
    public Transform target;
    public int maxIteration = 10;
    private float hold = 0.1f ;
    private float step = 1f;
    public IKTypeCalc typeIK;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        IKSolution();
    }

    void IKSolution()
    {
        if (joints == null && target == null)
            return;

        switch(typeIK)
        {
            case (IKTypeCalc.JACOBIAN):
                IKJacobian();
            break;
            default:
                break;
        }
    }

    void IKJacobian()
    {
        if (joints == null && target == null) 
            return;

        Vector3 end = joints[joints.ToArray().Length - 1].position;

        while(Vector3.Distance(end,target.position)> hold)
        {
            Matrix4x4 jacobian = CalcJacobian();

            Vector3 dirTarget = (target.position - end).normalized;

            Vector3 delta = dirTarget * hold;

            for (int i = 0; i < joints.ToArray().Length; i++)
            {
                Vector3 position = joints[i].position;
                Vector3 joindEndEffect = end - position;

                Quaternion rotation = Quaternion.FromToRotation(joindEndEffect, delta);
                joints[i].rotation *= rotation;

                end = joints[joints.ToArray().Length - 1].position;
            }
        }
    }

    Matrix4x4 CalcJacobian()
    {
        Matrix4x4 jacobian = new Matrix4x4();

        for (int i = 0; i < 3; i++)
        {
            Vector3 jointPosition = joints[i].position;
            Vector3 end = joints[joints.ToArray().Length - 1].position;
            Vector3 joinEndEffect = end - jointPosition;
            jacobian.SetColumn(i, Vector3.Cross(Vector3.up, joinEndEffect).normalized);
        }
        return jacobian;
    }
}
