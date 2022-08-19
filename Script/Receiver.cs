using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using System;
using System.Collections.Concurrent;
using System.Threading;

public class Receiver : MonoBehaviour
{
    NetMQ.Sockets.SubscriberSocket receive;
    //Initial parameter
    int co1=0;
    int co2=0;
    int co3=0;
    int co4=0;
    int co5=0;
    int co6=0;
    int co7=0;
    int co8=0;
    
    //Tool 1
    public static Vector3 tool_1_pos= new Vector3(0f,500f,10f);
    public static Vector3 tool_1_ang= new Vector3(0f,0f,0f);
    float t1_min_y_history1=500f;
    float t1_min_y_history2=500f;
    float t1_min_y_history3=500f;
    float t1_min_y_history4=500f;
    
    
    //Tool 2
    public static Vector3 tool_2_pos= new Vector3(0f,500f,30f);
    public static Vector3 tool_2_ang= new Vector3(0f,0f,0f);
    float t2_min_y_history1=500f;
    float t2_min_y_history2=500f;
    float t2_min_y_history3=500f;
    float t2_min_y_history4=500f;
    
    //Tool 3
    public static Vector3 tool_3_pos= new Vector3(0f,500f,50f);
    public static Vector3 tool_3_ang= new Vector3(0f,0f,0f);

    //Tool 4
    public static Vector3 tool_4_pos= new Vector3(0f,500f,70f);
    public static Vector3 tool_4_ang= new Vector3(0f,0f,0f);

    //Tool 5
    public static Vector3 tool_5_pos= new Vector3(0f,500f,90f);
    public static Vector3 tool_5_ang= new Vector3(0f,0f,0f);

    //Tool 6
    public static Vector3 tool_6_pos= new Vector3(0f,500f,110f);
    public static Vector3 tool_6_ang= new Vector3(0f,0f,0f);

    //Tool 7
    public static Vector3 tool_7_pos= new Vector3(0f,500f,130f);
    public static Vector3 tool_7_ang= new Vector3(0f,0f,0f);

    //Tool 8
    public static Vector3 tool_8_pos= new Vector3(0f,500f,150f);
    public static Vector3 tool_8_ang= new Vector3(0f,0f,0f);
    
    
    // Start is called before the first frame update
    void Start()
    {
        receive = new SubscriberSocket();
        receive.Connect("tcp://127.0.0.1:5555");
        receive.Subscribe("tool");
    }

    // Update is called once per frame
    void Update()
    {
        co1++;
        co2++;
        co3++;
        co4++;
        co5++;
        co6++;
        co7++;
        co8++;
        List<string> msg = null;
        //Obtain the data
        bool status = receive.TryReceiveMultipartStrings(TimeSpan.FromMilliseconds(25),ref msg);
        if (status == true)
        {   
            //Receive the data
            string data = msg[1];
            string[] tool_arry = data.Split("!");
            //Split the data and iterate each tools
            foreach(string tool_data in tool_arry)
            {
                if (tool_data==""){break;}
                string[] data_arry = tool_data.Split("_");
                double xk = Convert.ToDouble(data_arry[0]);
                double xb = Convert.ToDouble(data_arry[1]);
                double zk = Convert.ToDouble(data_arry[2]);
                double zb = Convert.ToDouble(data_arry[3]);
                float min_y = Convert.ToSingle(data_arry[4])-Convert.ToSingle(10*Math.Cos(Math.Atan(Math.Sqrt(xk * xk + zk * zk))));
                int index = Convert.ToInt32(data_arry[5])+1;

                float rz = Convert.ToSingle(Math.Atan(xk) * (-180 / Math.PI));
                float rx = Convert.ToSingle(Math.Atan(zk) * (180 / Math.PI));
                double height = 150 * Math.Cos(Math.Atan(Math.Sqrt(xk * xk + zk * zk)));
                double x = xk * height + xb;
                double z = zk * height + zb;

                //Dispatch the data
                if (index==1){
                    float y_offset=min_y*0.5f+t1_min_y_history1*0.25f+t1_min_y_history2*0.125f+t1_min_y_history3*0.1f+t1_min_y_history4*0.025f;
                    co1=0;
                    tool_1_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+y_offset, Convert.ToSingle(z));
                    tool_1_ang = new Vector3(rx, 0f, rz);
                    
                    t1_min_y_history4=t1_min_y_history3; //Oldest data
                    t1_min_y_history3=t1_min_y_history2;
                    t1_min_y_history2=t1_min_y_history1;
                    t1_min_y_history1=min_y;             //Nearest data
                    

                    Debug.Log("Tool1 "+y_offset);
                }
                if (index==2){
                    float y_offset=min_y*0.5f+t2_min_y_history1*0.25f+t2_min_y_history2*0.125f+t2_min_y_history3*0.1f+t2_min_y_history4*0.025f;
                    co2=0;
                    tool_2_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_2_ang = new Vector3(rx, 0f, rz);

                    t2_min_y_history4=t2_min_y_history3; //Oldest data
                    t2_min_y_history3=t2_min_y_history2;
                    t2_min_y_history2=t2_min_y_history1;
                    t2_min_y_history1=min_y;             //Nearest data

                    Debug.Log("Tool2 "+y_offset);
                }
                if (index==3){
                    co3=0;
                    tool_3_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_3_ang = new Vector3(rx, 0f, rz);
                }
                if (index==4){
                    co4=0;
                    tool_4_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_4_ang = new Vector3(rx, 0f, rz);
                }
                if (index==5){
                    co5=0;
                    tool_5_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_5_ang = new Vector3(rx, 0f, rz);
                }
                if (index==6){
                    co6=0;
                    tool_6_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_6_ang = new Vector3(rx, 0f, rz);
                }
                if (index==7){
                    co7=0;
                    tool_7_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_7_ang = new Vector3(rx, 0f, rz);
                }
                if (index==8){
                    co8=0;
                    tool_8_pos = new Vector3(Convert.ToSingle(x), Convert.ToSingle(height)+min_y, Convert.ToSingle(z));
                    tool_8_ang = new Vector3(rx, 0f, rz);
                }
            }

        //Initial the position    
        }
        if (co1>30)
        {
            tool_1_pos = new Vector3(0f,500f,10f);
            tool_1_ang= new Vector3(0f,0f,0f);
        }
        if (co2>30)
        {
            tool_2_pos = new Vector3(0f,500f,30f);
            tool_2_ang= new Vector3(0f,0f,0f);
        }
        if (co3>30)
        {
            tool_3_pos = new Vector3(0f,500f,50f);
            tool_3_ang= new Vector3(0f,0f,0f);
        }
        if (co4>30)
        {
            tool_4_pos = new Vector3(0f,500f,70f);
            tool_4_ang= new Vector3(0f,0f,0f);
        }
        if (co5>30)
        {
            tool_5_pos = new Vector3(0f,500f,90f);
            tool_5_ang= new Vector3(0f,0f,0f);
        }
        if (co6>30)
        {
            tool_6_pos = new Vector3(0f,500f,110f);
            tool_6_ang= new Vector3(0f,0f,0f);
        }
        if (co7>30)
        {
            tool_7_pos = new Vector3(0f,500f,130f);
            tool_7_ang= new Vector3(0f,0f,0f);
        }
        if (co8>30)
        {
            tool_8_pos = new Vector3(0f,500f,150f);
            tool_8_ang= new Vector3(0f,0f,0f);
        }
    }

    private void OnDestroy()
    {
        receive.Dispose();
        NetMQConfig.Cleanup(false);
    }
}
