using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum AxisOrder
{
    XYZ, XZY, YXZ, YZX, ZXY, ZYX, None
}

public static class Load_Init_Pos
{
    public static float[,] Read_CSV(string filename)
    {
        var textfile = Resources.Load<TextAsset>(filename);
        string raw_data = textfile.ToString();
        string[] lines = raw_data.Trim().Split('\n');
        int n_pose = lines.Length;
        Debug.Log(n_pose);
        int n_data = lines[0].Trim().Split(',').Length;
        Debug.Log(n_data);

        float[,] pose_rotations = new float[n_pose,n_data];
        for(int i = 0; i < n_pose; i++)
        {
            string[] data = lines[i].Trim().Split(',');
            for (int j = 0; j < n_data; j++)
            {
                float.TryParse(data[j], out pose_rotations[i, j]);
                
            }
        }

        return pose_rotations;
    }


    public static Quaternion BvhToUnityRotation(Vector3 eulerAngles, AxisOrder rotationOrder)
    {
        // BVH's x+ axis is Unity's left (x-)
        var xRot = Quaternion.AngleAxis(-eulerAngles.x, Vector3.left);
        // Unity & BVH agree on the y & z axes
        var yRot = Quaternion.AngleAxis(-eulerAngles.y, Vector3.up);
        var zRot = Quaternion.AngleAxis(-eulerAngles.z, Vector3.forward);

        switch (rotationOrder)
        {
            // Reproduce rotation order (no need for parentheses - it's associative)
            case AxisOrder.XYZ: return xRot * yRot * zRot;
            case AxisOrder.XZY: return xRot * zRot * yRot;
            case AxisOrder.YXZ: return yRot * xRot * zRot;
            case AxisOrder.YZX: return yRot * zRot * xRot;
            case AxisOrder.ZXY: return zRot * xRot * yRot;
            case AxisOrder.ZYX: return zRot * yRot * xRot;
        }

        return Quaternion.identity;
    }

    public static Quaternion ToQuaternion(Vector3 angles) // yaw (Z), pitch (Y), roll (X)
    {
        float yaw = angles.z;
        float pitch = angles.y;
        float roll = angles.x;

        // Abbreviations for the various angular functions
        float cy = Mathf.Cos(yaw * 0.5f);
        float sy = Mathf.Sin(yaw * 0.5f);
        float cp = Mathf.Cos(pitch * 0.5f);
        float sp = Mathf.Sin(pitch * 0.5f);
        float cr = Mathf.Cos(roll * 0.5f);
        float sr = Mathf.Sin(roll * 0.5f);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

}
