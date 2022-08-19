using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Receiver;

public class tool4 : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        transform.position = Receiver.tool_4_pos;
        transform.eulerAngles = Receiver.tool_4_ang;
    }
}
