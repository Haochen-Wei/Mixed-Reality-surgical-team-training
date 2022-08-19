using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Receiver;

public class tool3 : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        transform.position = Receiver.tool_3_pos;
        transform.eulerAngles = Receiver.tool_3_ang;
    }
}
