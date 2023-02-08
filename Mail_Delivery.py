# by Rayhaneh Behravesh & Antoine Minjon
#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Float64MultiArray
import numpy as np
import colorsys
 
 
class BayesLoc:
   def __init__(self, p0, colour_codes, colour_map):
       self.colour_sub = rospy.Subscriber(
           "mean_img_rgb", Float64MultiArray, self.colour_callback
       )
       self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
       self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
       self.line_index=0
       self.num_states = len(p0)
       self.colour_codes = colour_codes
       self.colour_map = colour_map
       self.probability = p0
       self.state_prediction = np.array([1/11]*11)
 
       self.cur_colour = np.array([0,0,0])  # most recent measured colour
       self.model=np.zeros(shape=(len(p0),len(p0)))
       self.measure=np.zeros(len(colour_codes))
       self.traverse = [6]
       self.prev_col = None
 
   def colour_callback(self, msg):
       """
       callback function that receives the most recent colour measurement from the camera.
       """
      
       self.cur_colour = np.array(msg.data)  # [r, g, b]
       colour_list = ["red","green","blue","yellow","line","None"]
       #change to hsv
 h_cur, s_cur, v_cur = colorsys.rgb_to_hsv(self.cur_colour[0]/255,self.cur_colour[1]/255,self.cur_colour[2]/255)
       #put in array
       self.cur_colour = np.array([h_cur, s_cur,v_cur])
       min = 1000
       val = 0
       for i in range(5):
          diff =  np.linalg.norm(colour_codes[i]-self.cur_colour)
          if diff<min:
           min = diff
           val = i
       # loop ended we can now detect what colour were on
       col_meas = colour_list[val]
       self.traverse.append(col_meas)
       self.prev_col = self.traverse[0] #initially index 6
       self.traverse.pop(0) # [6,new] ->> [new] - next round ->> [new,newnew]->>[newnew]
      
       if col_meas != self.prev_col and self.prev_col == 'line':
           print(self.state_prediction)
           self.state_predict()
           #we update the state based on the colour we measured
           self.state_update(col_meas)
           max_prob = np.max(self.state_prediction)
           #argmax returns indeces of max element in array. self.state_pred is probability. our path starts at 2 so every index (from 0) is added two to it
           max_prob_ind = np.argmax(self.state_prediction)+2
           print(max_prob_ind)
           print(" ")
 
           if int(max_prob_ind) in [2,8,3] and max_prob>0.6: #checking to see if the value of the max index is of the 3 stops we want to pause at
               rospy.sleep(2)
       self.prev_col = col_meas
       print(col_meas)#print detected colour
      
       #print(self.cur_colour)
 
   def line_callback(self, msg):
    
       self.line_index= msg.data
       error=self.line_index-320
       d=(error-Xk)
       Xk=error
       inte+=error
       correction=-error*0.003 - d*0.007 - inte*0.0001
       twist.linear.x=0.05
       c=0
       if self.prev_col != 'line':
           twist.angular.z=0
           inte=0
           self.cmd_pub.publish(self.twist)
              
       else:
           twist.angular.z=correction
           self.cmd_pub.publish(self.twist)
          
          
 
 
    
 
   def wait_for_colour(self):
       """Loop until a colour is received."""
       rate = rospy.Rate(100)
       while not rospy.is_shutdown() and self.cur_colour is None:
           rate.sleep()
 
   def state_model(self, u):
       p0 = np.ones_like(colour_map) / len(colour_map)
       model=np.zeros(shape=(3,len(p0),len(p0)))
       for i in range (len(p0)):
           model[0][i][i-1]=1
           model[1][i][i]=1
          
           if i==len(p0)-1:
               model[2][i][0]=1
           else:
               model[2][i][i+1]=1
       if u<0:
           a=0
       elif u>0:
           a=2
       else:
           a=1
       self.model=model[a]
 
 
   def measurement_model(self, x):
       """
       Measurement model p(z_k | x_k = colour) - given the pixel intensity,
       what's the probability that of each possible colour z_k being observed?
       """
       if self.cur_colour is None:
           self.wait_for_colour()
 
       prob = np.zeros(len(colour_codes))
 
  
       s=0
       for i in range(len(colour_codes)):
           prob[i]=np.linalg.norm(x-colour_codes[i])
           s+=prob[i]
       prob=prob/s
       self.measure=prob
 
   def state_predict(self):
       rospy.loginfo("predicting state")
       
 
          
       self.state_prediction=np.matmul(self.model,self.state_prediction)
 
   def state_update(self):
       rospy.loginfo("updating state")
      
       M=[[0,0,0,1,0],[0,1,0,0,0],[0,0,1,0,0],[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[1,0,0,0,0],[0,0,0,1,0],[0,1,0,0,0],[0,0,1,0,0]]
       self.state_prediction=np.multiply(np.matmul(M,self.measure),self.state_prediction) #to solve dimension problems
       self.state_prediction=self.state_prediction/(sum(self.state_prediction))
 
 
 
if __name__ == "__main__":
 
   # This is the known map of offices by colour
   # 0: red, 1: green, 2: blue, 3: yellow, 4: line
   # current map starting at cell #2 and ending at cell #12
   colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]
 
   # TODO calibrate these RGB values to recognize when you see a colour
   # NOTE: you may find it easier to compare colour readings using a different
   # colour system, such as HSV (hue, saturation, value). To convert RGB to
   # HSV, use:
   # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
   colour_codes = np.array([
       [235, 73, 129],  # red
       [157, 181, 164],  # green
       [179, 166, 184],  # blue
       [176, 162, 153],  # yellow
       [172, 161, 167],  # line
   ])
    colour_codes = np.array([
   [92, 73, 89],  # red
   [26, 10, 66],  # green
   [80, 13, 69],  # blue
   [9, 10, 62],  # yellow
   [89, 11, 63],  # line
   ])
 
 
   # initial probability of being at a given office is uniform
   p0 = np.ones_like(colour_map) / len(colour_map)
 
   localizer = BayesLoc(p0, colour_codes, colour_map)
 
   rospy.init_node("final_project")
   rospy.sleep(0.5)
   rate = rospy.Rate(10)
  
   twist=Twist()
   c=-100
 
   inte=0
   d=0
   Xk=0
   print(np.linalg.norm(colour_codes[0]-colour_codes[4]))
   print(np.linalg.norm(colour_codes[1]-colour_codes[4]))
   print(np.linalg.norm(colour_codes[2]-colour_codes[4]))
   print(np.linalg.norm(colour_codes[3]-colour_codes[4]))
 
   for i in range(300):
       """
       TODO: complete this main loop by calling functions from BayesLoc, and
       adding your own high level and low level planning + control logic
       """
     
     
 
 
 
 
 
       '''
       if np.linalg.norm(localizer.cur_colour-colour_codes[0])<50 or np.linalg.norm(localizer.cur_colour-colour_codes[1])<12 or np.linalg.norm(localizer.cur_colour-colour_codes[2])<10 or np.linalg.norm(localizer.cur_colour-colour_codes[3])<7 and c==10:
           if c==10:
               twist.angular.z=0
               inte=0
               print('color')
               localizer.state_model(0.075)
               localizer.measurement_model(localizer.cur_colour)
               localizer.state_predict()
               localizer.state_update()
               c+=1       
               """ to make it stop at K"""
               if localizer.state_prediction[k]>0.6:
                   rospy.sleep(2)
           else:
               twist.angular.z=0
               inte=0
               print('color')
               c+=1
       else:
           twist.angular.z=correction/1.5
           print('line')
           c=0
      
       """ to make it stop at K"""
       if localizer.state_prediction[k]>0.6:
          
       '''
       localizer.cmd_pub.publish(twist)
       rate.sleep()
 
   rospy.loginfo("finished!")
   rospy.loginfo(localizer.probability)
]\
