using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collision : MonoBehaviour
{
    private bool trigger;

    void OnTriggerEnter(Collider other)
    {
        trigger=true;
    }
    void OnTriggerExit(Collider other)
    {
        trigger=false;
    }
    void OnGUI()
    {
        if (trigger==true)
        {GUI.Box(new Rect(10,10,100,30), "Collision");}
    }
}
