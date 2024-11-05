using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum IKTypeCalc
{
    JACOBIAN,
    CCD,
    CCDJACOBIAN,
    FABRIK
}

public class IKManager : MonoBehaviour
{
    [Header("Kinematic Joints")]
    public Transform[] joints;
    [Header("Target to Reach")]
    public Transform target;

    #region Jacobian variable
    [Header("Constraints Jacobian")]
    public float jointAngleLimit = 180.0f;
    public Vector3 rotationAxisConstraint;
    #endregion
    
    #region CCD variable
    public float threshold = 0.1f;
    public int maxIterations = 10;
    #endregion
    
    public IKTypeCalc typeIK;
    private IkCCD ikCCD;
    private IkJacobian ikJacobian;
    private IkFabricks ikFabricks;
    private IkCCDJacobian ikCCDJacobian;
    void Start()
    {
        ikCCD         = GetComponent<IkCCD>();
        ikJacobian    = GetComponent<IkJacobian>();
        ikFabricks    = GetComponent<IkFabricks>();
        ikCCDJacobian = GetComponent<IkCCDJacobian>();
        if (ikCCD == null) ikCCD = gameObject.AddComponent<IkCCD>();
        if (ikJacobian == null) ikJacobian = gameObject.AddComponent<IkJacobian>();
        if (ikFabricks == null) ikFabricks = gameObject.AddComponent<IkFabricks>();
        if (ikCCDJacobian == null) ikCCDJacobian = gameObject.AddComponent<IkCCDJacobian>();
    }
    
    void Update()
    {
        switch(typeIK)
        {
            case (IKTypeCalc.JACOBIAN):
                ikJacobian.IKJacobianFunc();
                break;
            case (IKTypeCalc.CCD):
                if(Vector3.Distance(joints[joints.Length -1].position,target.position)>threshold)
                    ikCCD.IkCCDFunc();
                break;
            case (IKTypeCalc.CCDJACOBIAN):
                ikCCDJacobian.IkCCDJacobianFunc();
                break;
            case (IKTypeCalc.FABRIK):
                ikFabricks.IkFabrikFunc();
                break;
            default:
                break;
        }
    }
}
