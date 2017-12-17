import numpy as np
import time
from pyglet.window.key import END
from test.test_support import bigaddrspacetest
from numpy import deg2rad

def PTPtoConfiguration(start_cfg, target_cfg, motiontype):
    """PTP path planning
    
    :param start_cfg: Current axis angle of the robot
    :type start_cfg: array of floats
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
   

    start_rad= start_cfg
    end_rad=target_cfg
    start=np.rad2deg(start_rad)
    end=np.rad2deg(end_rad)
    
    delta_S=end-start
    print "delta_s",delta_S
    print "start:",start
    print "end:",end
    #print delta_S
    v_max=[100.0,80.0,80.0,230.0,165.0,249.0]
    t_acc=0.1

    print motiontype
    a_max=[0.0,0.0,0.0,0.0,0.0,0.0]
    t1=[0.0,0.0,0.0,0.0,0.0,0.0]
    t2=[0.0,0.0,0.0,0.0,0.0,0.0]
    t3=[0.0,0.0,0.0,0.0,0.0,0.0]
    shape1=[0,0,0,0,0,0]
    for i in range(len(a_max)):
        a_max[i]=v_max[i]/t_acc
    for i in range(len(a_max)):
        t1[i]=v_max[i]/a_max[i]
        t2[i]=abs(delta_S[i])/v_max[i]
   
        if(t2[2]<t1[i]):
            t1[i]=np.sqrt(abs(delta_S[i])/a_max[i])
            t2[i]=t1[i]
            shape1[i]=1
        t3[i]=t1[i]+t2[i]
    
    
    trajectory = np.empty([100, 6])
#############################ASYNCHRONOUS##############################################
    if(motiontype=="A"):
        for i in range(len(a_max)):
            if(shape1[i]==1):
                t=0
                delta_t=t3[i]/100
                t=t+delta_t
                for j in range(50):
                    tra=0.5*a_max[i]*t*t
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t
                
                for j in range(50,100):
                    tra=(0.5*abs(delta_S[i]))+((abs(delta_S[i])/t1[i])*(t-t1[i]))-(0.5*a_max[i]*(t-t1[i])*(t-t1[i]))
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t                
            else:
                t=0
                delta_t=t3[i]/100 
                t=t+delta_t
                j=0
                while(t<t1[i]):
                    tra=0.5*a_max[i]*t*t
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    j=j+1                  
                    t=t+delta_t               
                while(t>=t1[i] and t<t2[i]):
                    tra=(0.5*t1[i]*v_max[i])+(v_max[i]*(t-t1[i]))
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    j=j+1
                    t=t+delta_t
                n=j
                for j in range(n+1,100):
                    tra=((v_max[i]*t2[i])-(0.5*v_max[i]*t1[i]))+v_max[i]*(t-t2[i])-(0.5*a_max[i]*(t-t2[i])*(t-t2[i]))
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    j=j+1
                    t=t+delta_t
 ############### SYNCHRONOUS##############################################                   
    elif(motiontype=="S"):
        v_temp=[0,0,0,0,0,0]
        v_max_new1=[0,0,0,0,0,0] 
        v_max_new2=[0,0,0,0,0,0] 
        t_acc=[0,0,0,0,0,0]
        t_flat=[0,0,0,0,0,0]
        big=0
        index=0
        for i in range(5):
            if(big>=t3[i]):
                big=big
            else:
                big=t3[i]
                index=i
        t_max=big        
        
        
        for i in range(6):
            if(shape1[i]==5):
                v_max_new1[i]=(abs(delta_S[i])/t_max)
                v_max_new2[i]=(abs(delta_S[i])/t_max)
            else:
                a=-(1/a_max[i])
                b=t_max
                c=-1*abs(delta_S[i])
                v_max_new1[i]=(-b+np.sqrt((b*b)-(4*a*c)))/(2*a)
                v_max_new2[i]=(-b+np.sqrt((b*b)-(4*a*c)))/(2*a)
           
        if(v_max[index]==v_max_new1[index]):
            v_temp=v_max_new1
        else:
            v_temp=v_max_new2
     
        
        for i in range(6):
            if(shape1[i]==0):
                t_acc[i]=v_temp[i]/a_max[i]
            else:
                t_acc[i]=t_max*0.5
     
        
        for i in range(6):
            if(shape1[i]==0):
                t_flat[i]=t_max-(2*t_acc[i])
            else:
                t_flat[i]=0
     
        
        for i in range(6):
            if(shape1[i]==0):
                t=0
                delta_t=t_max*0.01
                j=0
                t=t+delta_t
                while(t<=t_acc[i]):
                    tra=0.5*a_max[i]*t*t
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t 
                    j=j+1          
                while(t>t_acc[i] and t<=(t_acc[i]+t_flat[i])):
                    tra=(0.5*t_acc[i]*v_temp[i])+(v_temp[i]*(t-t_acc[i]))
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t 
                    j=j+1          
                    
                while((t>(t_acc[i]+t_flat[i])) and (t<=t_max+delta_t) and j<100):
                    tra=((v_temp[i]*(t_acc[i]+t_flat[i]))-(0.5*v_temp[i]*t_acc[i]))+v_temp[i]*(t-(t_acc[i]+t_flat[i]))-(0.5*a_max[i]*(t-(t_acc[i]+t_flat[i]))*(t-(t_acc[i]+t_flat[i])))
                    
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t  
                    j=j+1         
            else:

                t=0
                delta_t=t_max*0.01
                t=t+delta_t
                
                for j in range(51):            
                    
                    tra=0.5*v_temp[i]*t                   
                    
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    t=t+delta_t 
                            
                for j in range(50,100):                  
                
                    tra=(0.5*t_acc[i]*v_temp[i])+((t_acc[i]*v_temp[i])-(v_temp[i]*(t_max-t)))
                    if(delta_S[i]>0):
                        trajectory[j,i]=tra
                    elif(delta_S[i]<0):
                        trajectory[j,i]=-tra
                    elif(delta_S[i]==0):
                        trajectory[j,i]=0
                    if(j==99):
                        trajectory[j,i]=end[i]
                    
                    t=t+delta_t 
                    
                              
    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    
    #diff = target_cfg - start_cfg
    #delta = diff / 100.0  
    
    #for i in xrange(100):
    #    trajectory[i] = start_cfg + (i*delta)
        
    #trajectory[99] = target_cfg
    for i in range(100):
        
        trajectory[i]=trajectory[i]+start
        
    
    return (trajectory)


def Move(robot, trajectory):
    
    for i in range(100):
        n=deg2rad(trajectory[i])

        robot.SetDOFValues(n)
        time.sleep(0.01)

