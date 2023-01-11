{\rtf1\ansi\ansicpg1252\cocoartf2639
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\froman\fcharset0 Times-Roman;\f1\fmodern\fcharset0 CourierNewPSMT;}
{\colortbl;\red255\green255\blue255;\red0\green0\blue0;\red15\green112\blue1;\red255\green255\blue255;
\red0\green0\blue255;\red144\green1\blue18;\red19\green118\blue70;}
{\*\expandedcolortbl;;\cssrgb\c0\c0\c0;\cssrgb\c0\c50196\c0;\cssrgb\c100000\c100000\c100000;
\cssrgb\c0\c0\c100000;\cssrgb\c63922\c8235\c8235;\cssrgb\c3529\c52549\c34510;}
\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\deftab720
\pard\pardeftab720\qj\partightenfactor0

\f0\fs24 \cf0 \expnd0\expndtw0\kerning0
\
\pard\pardeftab720\partightenfactor0

\f1 \cf3 \cb4 #!/usr/bin/env python
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf5 import\cf0  rospy
\f0 \

\f1 \cf5 import\cf0  math
\f0 \

\f1 \cf5 from\cf0  geometry_msgs.msg \cf5 import\cf0  Twist
\f0 \

\f1 \cf5 from\cf0  std_msgs.msg \cf5 import\cf0  UInt32, Float64MultiArray
\f0 \

\f1 \cf5 import\cf0  numpy \cf5 as\cf0  np
\f0 \

\f1 \cf5 import\cf0  colorsys
\f0 \
\'a0\
\'a0\

\f1 \cf5 class\cf0  BayesLoc:
\f0 \

\f1 \'a0\'a0\'a0\cf5 def\cf0  __init__(self, p0, colour_codes, colour_map):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .colour_sub = rospy.Subscriber(
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 "mean_img_rgb"\cf0 , Float64MultiArray, \cf5 self\cf0 .colour_callback
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .line_sub = rospy.Subscriber(\cf6 "line_idx"\cf0 , UInt32, \cf5 self\cf0 .line_callback)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cmd_pub = rospy.Publisher(\cf6 "cmd_vel"\cf0 , Twist, queue_size=\cf7 1\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .line_index=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .num_states = len(p0)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .colour_codes = colour_codes
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .colour_map = colour_map
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .probability = p0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_prediction = np.array([\cf7 1\cf0 /\cf7 11\cf0 ]*\cf7 11\cf0 )
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cur_colour = np.array([\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ])\'a0 \cf3 # most recent measured colour
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .model=np.zeros(shape=(len(p0),len(p0)))
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .measure=np.zeros(len(colour_codes))
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .traverse = [\cf7 6\cf0 ]
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .prev_col = \cf5 None
\f0 \cf0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  colour_callback(self, msg):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0callback function that receives the most recent colour measurement from the camera.
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf0 \'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cur_colour = np.array(msg.data)\'a0 \cf3 # [r, g, b]
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0colour_list = [\cf6 "red"\cf0 ,\cf6 "green"\cf0 ,\cf6 "blue"\cf0 ,\cf6 "yellow"\cf0 ,\cf6 "line"\cf0 ,\cf6 "None"\cf0 ]
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 #change to hsv
\f0 \cf0 \

\f1 \'a0h_cur, s_cur, v_cur = colorsys.rgb_to_hsv(\cf5 self\cf0 .cur_colour[\cf7 0\cf0 ]/\cf7 255\cf0 ,\cf5 self\cf0 .cur_colour[\cf7 1\cf0 ]/\cf7 255\cf0 ,\cf5 self\cf0 .cur_colour[\cf7 2\cf0 ]/\cf7 255\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 #put in array
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cur_colour = np.array([h_cur, s_cur,v_cur])
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0min = \cf7 1000
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0val = \cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 for\cf0  i \cf5 in\cf0  range(\cf7 5\cf0 ):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0diff =\'a0 np.linalg.norm(colour_codes[i]-\cf5 self\cf0 .cur_colour)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  diff<min:
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0min = diff
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0val = i
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 # loop ended we can now detect what colour were on
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0col_meas = colour_list[val]
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .traverse.append(col_meas)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .prev_col = \cf5 self\cf0 .traverse[\cf7 0\cf0 ] \cf3 #initially index 6
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .traverse.pop(\cf7 0\cf0 ) \cf3 # [6,new] ->> [new] - next round ->> [new,newnew]->>[newnew]
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  col_meas != \cf5 self\cf0 .prev_col \cf5 and\cf0  \cf5 self\cf0 .prev_col == \cf6 'line'\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print(\cf5 self\cf0 .state_prediction)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_predict()
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 #we update the state based on the colour we measured
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_update(col_meas)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0max_prob = np.max(\cf5 self\cf0 .state_prediction)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 #argmax returns indeces of max element in array. self.state_pred is probability. our path starts at 2 so every index (from 0) is added two to it
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0max_prob_ind = np.argmax(\cf5 self\cf0 .state_prediction)+\cf7 2
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print(max_prob_ind)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print(\cf6 " "\cf0 )
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  int(max_prob_ind) \cf5 in\cf0  [\cf7 2\cf0 ,\cf7 8\cf0 ,\cf7 3\cf0 ] \cf5 and\cf0  max_prob>\cf7 0.6\cf0 : \cf3 #checking to see if the value of the max index is of the 3 stops we want to pause at
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0rospy.sleep(\cf7 2\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .prev_col = col_meas
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0print(col_meas)\cf3 #print detected colour
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf3 #print(self.cur_colour)
\f0 \cf0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  line_callback(self, msg):
\f0 \

\f1 \'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .line_index= msg.data
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0error=\cf5 self\cf0 .line_index-\cf7 320
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0d=(error-Xk)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0Xk=error
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0inte+=error
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0correction=-error*\cf7 0.003\cf0  - d*\cf7 0.007\cf0  - inte*\cf7 0.0001
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.linear.x=\cf7 0.05
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0c=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  \cf5 self\cf0 .prev_col != \cf6 'line'\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.angular.z=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0inte=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cmd_pub.publish(\cf5 self\cf0 .twist)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 else\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.angular.z=correction
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .cmd_pub.publish(\cf5 self\cf0 .twist)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \
\'a0\
\'a0\

\f1 \'a0\'a0\'a0\'a0
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  wait_for_colour(self):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """Loop until a colour is received."""
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0rate = rospy.Rate(\cf7 100\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 while\cf0  \cf5 not\cf0  rospy.is_shutdown() \cf5 and\cf0  \cf5 self\cf0 .cur_colour \cf5 is\cf0  \cf5 None\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0rate.sleep()
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  state_model(self, u):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0p0 = np.ones_like(colour_map) / len(colour_map)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0model=np.zeros(shape=(\cf7 3\cf0 ,len(p0),len(p0)))
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 for\cf0  i \cf5 in\cf0  range (len(p0)):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0model[\cf7 0\cf0 ][i][i-\cf7 1\cf0 ]=\cf7 1
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0model[\cf7 1\cf0 ][i][i]=\cf7 1
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  i==len(p0)-\cf7 1\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0model[\cf7 2\cf0 ][i][\cf7 0\cf0 ]=\cf7 1
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 else\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0model[\cf7 2\cf0 ][i][i+\cf7 1\cf0 ]=\cf7 1
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  u<\cf7 0\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0a=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 elif\cf0  u>\cf7 0\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0a=\cf7 2
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 else\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0a=\cf7 1
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .model=model[a]
\f0 \
\pard\pardeftab720\qj\partightenfactor0
\cf0 \'a0\
\pard\pardeftab720\partightenfactor0
\cf0 \'a0\
\pard\pardeftab720\partightenfactor0

\f1 \cf0 \'a0\'a0\'a0\cf5 def\cf0  measurement_model(self, x):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0Measurement model p(z_k | x_k = colour) - given the pixel intensity,
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0what's the probability that of each possible colour z_k being observed?
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf0 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 if\cf0  \cf5 self\cf0 .cur_colour \cf5 is\cf0  \cf5 None\cf0 :
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .wait_for_colour()
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0prob = np.zeros(len(colour_codes))
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 TODO\cf6 : You need to compute the probability of states. You should return a 1x5 np.array
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0and the reference RGB values of each colour (self.ColourCodes).
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf0 \'a0\'a0\'a0\'a0\'a0\'a0\'a0s=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 for\cf0  i \cf5 in\cf0  range(len(colour_codes)):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0prob[i]=np.linalg.norm(x-colour_codes[i])
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0s+=prob[i]
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0prob=prob/s
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .measure=prob
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  state_predict(self):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0rospy.loginfo(\cf6 "predicting state"\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \
\pard\pardeftab720\partightenfactor0

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 TODO\cf6 : Complete the state prediction function: update
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0self.state_prediction with the predicted probability of being at each
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0state (office)
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \
\pard\pardeftab720\qj\partightenfactor0
\cf0 \'a0\
\pard\pardeftab720\qj\partightenfactor0

\f1 \cf0 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_prediction=np.matmul(\cf5 self\cf0 .model,\cf5 self\cf0 .state_prediction)
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 def\cf0  state_update(self):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0rospy.loginfo(\cf6 "updating state"\cf0 )
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 TODO\cf6 : Complete the state update function: update self.probabilities
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0with the probability of being at each state R G B Y
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0M=[[\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ],[\cf7 0\cf0 ,\cf7 0\cf0 ,\cf7 1\cf0 ,\cf7 0\cf0 ,\cf7 0\cf0 ]]
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_prediction=np.multiply(np.matmul(M,\cf5 self\cf0 .measure),\cf5 self\cf0 .state_prediction) \cf3 #to solve dimension problems
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 self\cf0 .state_prediction=\cf5 self\cf0 .state_prediction/(sum(\cf5 self\cf0 .state_prediction))
\f0 \
\'a0\
\'a0\
\'a0\

\f1 \cf5 if\cf0  __name__ == \cf6 "__main__"\cf0 :
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf3 # This is the known map of offices by colour
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # 0: red, 1: green, 2: blue, 3: yellow, 4: line
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # current map starting at cell #2 and ending at cell #12
\f0 \cf0 \

\f1 \'a0\'a0\'a0colour_map = [\cf7 3\cf0 , \cf7 0\cf0 , \cf7 1\cf0 , \cf7 2\cf0 , \cf7 2\cf0 , \cf7 0\cf0 , \cf7 1\cf0 , \cf7 2\cf0 , \cf7 3\cf0 , \cf7 0\cf0 , \cf7 1\cf0 ]
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf3 # \cf5 TODO\cf3  calibrate these RGB values to recognize when you see a colour
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # \cf5 NOTE\cf3 : you may find it easier to compare colour readings using a different
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # colour system, such as HSV (hue, saturation, value). To convert RGB to
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # HSV, use:
\f0 \cf0 \

\f1 \'a0\'a0\'a0\cf3 # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
\f0 \cf0 \

\f1 \'a0\'a0\'a0colour_codes = np.array([
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0[\cf7 235\cf0 , \cf7 73\cf0 , \cf7 129\cf0 ],\'a0 \cf3 # red
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0[\cf7 157\cf0 , \cf7 181\cf0 , \cf7 164\cf0 ],\'a0 \cf3 # green
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0[\cf7 179\cf0 , \cf7 166\cf0 , \cf7 184\cf0 ],\'a0 \cf3 # blue
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0[\cf7 176\cf0 , \cf7 162\cf0 , \cf7 153\cf0 ],\'a0 \cf3 # yellow
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0[\cf7 172\cf0 , \cf7 161\cf0 , \cf7 167\cf0 ],\'a0 \cf3 # line
\f0 \cf0 \

\f1 \'a0\'a0\'a0])
\f0 \

\f1 \'a0\'a0\'a0\'a0colour_codes = np.array([
\f0 \

\f1 \'a0\'a0\'a0[\cf7 92\cf0 , \cf7 73\cf0 , \cf7 89\cf0 ],\'a0 \cf3 # red
\f0 \cf0 \

\f1 \'a0\'a0\'a0[\cf7 26\cf0 , \cf7 10\cf0 , \cf7 66\cf0 ],\'a0 \cf3 # green
\f0 \cf0 \

\f1 \'a0\'a0\'a0[\cf7 80\cf0 , \cf7 13\cf0 , \cf7 69\cf0 ],\'a0 \cf3 # blue
\f0 \cf0 \

\f1 \'a0\'a0\'a0[\cf7 9\cf0 , \cf7 10\cf0 , \cf7 62\cf0 ],\'a0 \cf3 # yellow
\f0 \cf0 \

\f1 \'a0\'a0\'a0[\cf7 89\cf0 , \cf7 11\cf0 , \cf7 63\cf0 ],\'a0 \cf3 # line
\f0 \cf0 \

\f1 \'a0\'a0\'a0])
\f0 \
\'a0\
\'a0\

\f1 \'a0\'a0\'a0\cf3 # initial probability of being at a given office is uniform
\f0 \cf0 \

\f1 \'a0\'a0\'a0p0 = np.ones_like(colour_map) / len(colour_map)
\f0 \
\'a0\

\f1 \'a0\'a0\'a0localizer = BayesLoc(p0, colour_codes, colour_map)
\f0 \
\'a0\

\f1 \'a0\'a0\'a0rospy.init_node(\cf6 "final_project"\cf0 )
\f0 \

\f1 \'a0\'a0\'a0rospy.sleep(\cf7 0.5\cf0 )
\f0 \

\f1 \'a0\'a0\'a0rate = rospy.Rate(\cf7 10\cf0 )
\f0 \

\f1 \'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0twist=Twist()
\f0 \

\f1 \'a0\'a0\'a0c=-\cf7 100
\f0 \cf0 \
\'a0\

\f1 \'a0\'a0\'a0inte=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0d=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0Xk=\cf7 0
\f0 \cf0 \

\f1 \'a0\'a0\'a0print(np.linalg.norm(colour_codes[\cf7 0\cf0 ]-colour_codes[\cf7 4\cf0 ]))
\f0 \

\f1 \'a0\'a0\'a0print(np.linalg.norm(colour_codes[\cf7 1\cf0 ]-colour_codes[\cf7 4\cf0 ]))
\f0 \

\f1 \'a0\'a0\'a0print(np.linalg.norm(colour_codes[\cf7 2\cf0 ]-colour_codes[\cf7 4\cf0 ]))
\f0 \

\f1 \'a0\'a0\'a0print(np.linalg.norm(colour_codes[\cf7 3\cf0 ]-colour_codes[\cf7 4\cf0 ]))
\f0 \
\'a0\

\f1 \'a0\'a0\'a0\cf5 for\cf0  i \cf5 in\cf0  range(\cf7 300\cf0 ):
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 """
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf5 TODO\cf6 : complete this main loop by calling functions from BayesLoc, and
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0adding your own high level and low level planning + control logic
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0"""
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0
\f0 \
\'a0\
\'a0\
\'a0\
\'a0\
\'a0\

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\cf6 '''
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0if np.linalg.norm(localizer.cur_colour-colour_codes[0])<50 or np.linalg.norm(localizer.cur_colour-colour_codes[1])<12 or np.linalg.norm(localizer.cur_colour-colour_codes[2])<10 or np.linalg.norm(localizer.cur_colour-colour_codes[3])<7 and c==10:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0if c==10:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.angular.z=0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0inte=0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print('color')
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0localizer.state_model(0.075)
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0localizer.measurement_model(localizer.cur_colour)
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0localizer.state_predict()
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0localizer.state_update()
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0c+=1\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0""" to make it stop at K"""
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0if localizer.state_prediction[k]>0.6:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0rospy.sleep(2)
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0else:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.angular.z=0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0inte=0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print('color')
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0c+=1
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0else:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0twist.angular.z=correction/1.5
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0print('line')
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0c=0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0""" to make it stop at K"""
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0if localizer.state_prediction[k]>0.6:
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0\'a0
\f0 \cf0 \

\f1 \cf6 \'a0\'a0\'a0\'a0\'a0\'a0\'a0'''
\f0 \cf0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0localizer.cmd_pub.publish(twist)
\f0 \

\f1 \'a0\'a0\'a0\'a0\'a0\'a0\'a0rate.sleep()
\f0 \
\'a0\

\f1 \'a0\'a0\'a0rospy.loginfo(\cf6 "finished!"\cf0 )
\f0 \

\f1 \'a0\'a0\'a0rospy.loginfo(localizer.probability)
\f0 \

\f1 ]\\
\f0 \
}