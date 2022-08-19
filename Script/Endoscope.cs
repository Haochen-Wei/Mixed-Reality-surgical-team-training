using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using System;
using System.Collections.Concurrent;
using System.Threading;

public class Endoscope : MonoBehaviour
{
    NetMQ.Sockets.SubscriberSocket endoscope;
    //Initial parameter
    
    //Tool 1
    public static Vector3 end_pos= new Vector3(63f,115f,225f);
    public static Vector3 end_ang= new Vector3(69f,-79f,15f);
    
    Vector3 current_pos= new Vector3();
    Vector3 pos_history1=new Vector3(63f,115f,225f);
    Vector3 pos_history2=new Vector3(63f,115f,225f);
    Vector3 pos_history3=new Vector3(63f,115f,225f);
    Vector3 pos_history4=new Vector3(63f,115f,225f);
    
    // Vector3 current_ang= new Vector3();
    // Vector3 ang_history1=new Vector3(69f,-79f,15f);
    // Vector3 ang_history2=new Vector3(69f,-79f,15f);
    // Vector3 ang_history3=new Vector3(69f,-79f,15f);
    // Vector3 ang_history4=new Vector3(69f,-79f,15f);
    
    // Start is called before the first frame update
    void Start()
    {
        endoscope = new SubscriberSocket();
        endoscope.Connect("tcp://127.0.0.1:5555");
        endoscope.Subscribe("end");
        transform.position = end_pos;
        transform.eulerAngles = end_ang;
    }

    // Update is called once per frame
    void Update()
    {
        List<string> msg = null;
        //Obtain the data
        bool status = endoscope.TryReceiveMultipartStrings(TimeSpan.FromMilliseconds(30),ref msg);
        if (status == true)
        {   
            //Receive the data
            string data = msg[1];
            string[] E_arry = data.Split("!");
            //Unpack the data to transform matrix
            
            float a = Convert.ToSingle(E_arry[0]);
            float b = Convert.ToSingle(E_arry[1]);
            float c = Convert.ToSingle(E_arry[2]);
            float d = Convert.ToSingle(E_arry[3]);
            float e = Convert.ToSingle(E_arry[4]);
            float f = Convert.ToSingle(E_arry[5]);
            float g = Convert.ToSingle(E_arry[6]);
            float h = Convert.ToSingle(E_arry[7]);
            float i = Convert.ToSingle(E_arry[8]);
            float j = Convert.ToSingle(E_arry[9]);
            float k = Convert.ToSingle(E_arry[10]);
            float l = Convert.ToSingle(E_arry[11]);
            float m = Convert.ToSingle(E_arry[12]);
            float n = Convert.ToSingle(E_arry[13]);
            float o = Convert.ToSingle(E_arry[14]);
            float p = Convert.ToSingle(E_arry[15]);

            var matrix = Matrix4x4.zero;
            matrix[0,0]=a;
            matrix[0,1]=-b;
            matrix[0,2]=c;
            matrix[0,3]=d;
            matrix[1,0]=-e;
            matrix[1,1]=f;
            matrix[1,2]=-g;
            matrix[1,3]=-h;
            matrix[2,0]=i;
            matrix[2,1]=-j;
            matrix[2,2]=k;
            matrix[2,3]=l;
            matrix[3,0]=m;
            matrix[3,1]=-n;
            matrix[3,2]=o;
            matrix[3,3]=p;


            if(matrix[3,3]!=0)
            {
                current_pos.x=matrix[0,3]*1000;
                current_pos.y=matrix[1,3]*1000;
                current_pos.z=matrix[2,3]*1000;
                transform.position = current_pos*0.3f+pos_history1*0.2f+pos_history2*0.2f+pos_history3*0.2f+pos_history4*0.1f;//new Vector3(matrix[0,3]*1000, matrix[1,3]*1000, matrix[2,3]*1000);
                transform.eulerAngles = matrix.rotation.eulerAngles;

                pos_history4=pos_history3;           //Oldest data
                pos_history3=pos_history2;
                pos_history2=pos_history1;
                pos_history1=transform.position;     //Nearest data
            }  
        }   
    }

    private void OnDestroy()
    {
        endoscope.Dispose();
        NetMQConfig.Cleanup(false);
    }
}
