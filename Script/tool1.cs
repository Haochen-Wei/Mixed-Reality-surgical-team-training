using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Receiver;

public class tool1 : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        transform.position = Receiver.tool_1_pos;
        transform.eulerAngles = Receiver.tool_1_ang;
    }
}
