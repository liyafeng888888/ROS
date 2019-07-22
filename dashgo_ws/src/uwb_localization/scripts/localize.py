#!/usr/bin/env python  

'''
 Copyright (c) 2016, Juan Jimeno
 
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
'''
 
import rospy
import tf
import localization as lx
import serial

def parse_data(data_string):
    #rospy.loginfo("data_string is %s", data_string)    
    range_dict={}
    if data_string != None:
        current_key = ""
        #iterate message string without first and last character of message
	if data_string[0] == "c":
	  #rospy.loginfo("data_string is %s", data_string)
          for i in range(1,len(data_string)): 
            #anchor id is located at odd indexes
	     #rospy.loginfo("i is %s", i)  
	     if i==2:
                current_anchor = str(1)
                current_range = int(data_string[i],16)/1000.000
		#rospy.loginfo("current_range0 is %s", current_range)
		if float(current_range) < MAX_RANGE and float(current_range) > MIN_RANGE:
                    #populate dictionary
                    range_dict[current_anchor] = current_range
                else:
		    break      
             if i==3:
                current_anchor = str(2)
                current_range = int(data_string[i],16)/1000.000
		#rospy.loginfo("current_range1 is %s", current_range)
		if float(current_range) < MAX_RANGE and float(current_range) > MIN_RANGE:
                    #populate dictionary
                    range_dict[current_anchor] = current_range
                else:
		    break
            #anchor range is located at even indexes    
             if i==4:
                current_anchor = str(3)
                current_range = int(data_string[i],16)/1000.000
		#rospy.loginfo("current_range2 is %s", current_range)
                #filter out data that doesn't make sense
                if float(current_range) < MAX_RANGE and float(current_range) > MIN_RANGE:
                    #populate dictionary
                    range_dict[current_anchor] = current_range
		    #rospy.loginfo("current_range1 is %s", range_dict[str(2)])
		else:
		    break
         #if data_string[0] == "c": 
		#current_anchor = str(1)
                #current_range = data_string[2]
	    #if float(current_range) < MAX_RANGE and float(current_range) > MIN_RANGE:
                    #populate dictionary
                    #range_dict[current_anchor] = current_range
        return range_dict

def get_transforms(anchors):
    transform_dict = {}
    #iterate and get the transforms for each anchor
    for anchor in anchors:
        try:
            (trans,rot) = listener.lookupTransform('/map', anchor, rospy.Time(0))
            transform_dict[anchor] = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass         
               
    return transform_dict
    
def get_target_location(transforms, ranges):
    P = lx.Project(mode="3D",solver="LSE")
    
    #define anchor locations
    for anchor in transforms:        
        P.add_anchor(anchor, transforms[anchor])
    t, label = P.add_target()

    #define anchor ranges
    for anchor in ranges:       
        t.add_measure(anchor,ranges[anchor])

    P.solve()
    B = t.loc

    return {'x':B.x, 'y':B.y, 'z':B.z}
    
def read_serial_data():
    #PROTOCOL: "$TOTAL_DEVICES_AROUND,ADDRESS_1,RANGE_1,ADDRESS_2,RANGE_2,ADDRESS_N,RANGE_N,TOTAL_DEVICES_SENT\r\n"
    data_string = ""
    parsed_data_string = ""
    
    #tell Arduino to send the data
    #ser.write('+')
    # print "hello"
    #check the first character of the message
    #rospy.loginfo("This is info %s","ok by lyh")
    if ser.read() == 'm':
        while True:
            #rospy.loginfo("This is info %s","ok by lyh")
            c = ser.read()
            data_string += c
            # print data_string
            # print data_string
            #stop collecting characters at end of message
            if c == '\n':
                #omit start and end characters
                #rospy.loginfo("data_string is %s", data_string)   
                data_string = data_string.rstrip('\r\n')
                #rospy.loginfo("data_string is %s", data_string)
                try:
                    #parse anchor ids and ranges from message
                    parsed_data_string = data_string.split(' ')
                    total_anchors = 3 
                   #rospy.loginfo("parsed_data_string is %s", parsed_data_string)                  
                    #exit when there's not enough anchors around
                    if total_anchors < MIN_ANCHOR or total_anchors == 0:
                        duration = rospy.get_time() - start_time
                        #prevent from complaining on sensor boot up
                        if duration > 5:
                            rospy.logwarn("Not enough anchors. Make sure anchors are powered on or try moving around near the anchors.")
                        break
                    else:
                        return parsed_data_string                       
                except:
                    rospy.logwarn("Error parsing reading serial data")                 
                    pass                      
            # elif c == '$':
            #     break
if __name__ == '__main__':
    
    rospy.init_node('lips')
    listener = tf.TransformListener()
    start_time = rospy.get_time()
    #create rosparameters
    MIN_RANGE = rospy.get_param('/lips/min_range', 0.2)
    MAX_RANGE = rospy.get_param('/lips/max_range', 20.0)
    MIN_ANCHOR = rospy.get_param('/lips/min_anchor', 2)
    FRAME_ID = rospy.get_param('/lips/frame_id', 'uwb_tag')
    #SERIAL_PORT = rospy.get_param('/lips/serial_port', '/dev/ttyUSB0')
    SERIAL_PORT = rospy.get_param('/lips/serial_port', '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0')
    #rosparam logs just to make sure parameters kicked in
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/min_range'), MIN_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/max_range'), MAX_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/min_anchor'), MIN_ANCHOR)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/frame_id'), FRAME_ID)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/serial_sport'), SERIAL_PORT)
     
    ser = serial.Serial(SERIAL_PORT, 115200)
    rospy.loginfo("Connected to %s", ser.portstr)
    num=0
    #rangex=0
    #rangey=0
    #rangez=0
    while not rospy.is_shutdown():
        rospy.sleep
        #get the range of each anchor
	ranges={}
        
        ranges = parse_data(read_serial_data())
	#rospy.loginfo("current_range25 is %s", ranges.get('2'))
        #only perform calculations when there's valid r`anges from the anchors
        if ranges != None:
	    anchors = []          
            #populate anchor dictionary 
            for anchor in ranges:
                anchors.append(anchor)
                
            #get transforms of each anchor
            transforms = get_transforms(anchors)
            
            if(len(transforms) >= MIN_ANCHOR):
                try:
                    #perform trilateration using anchor ranges
                    pos = get_target_location(transforms, ranges)
		#    num = num + 1
          	#    rangex = rangex + pos['x']
	  	#    rangey = rangey + pos['y']
	  	#    rangez = rangez + pos['z']
                except:
                    rospy.logwarn("Localization Error")
		
	  	#rospy.loginfo("num is %s", num)
	  	#if num == 3:
          	#  pos['x']=rangex/3.000
	  	#  pos['y']=rangey/3.000
	   	#  pos['z']=rangez/3.000
	   	 #rospy.loginfo("range2 is %s", ranges.get('2'))
           	#  num=0
	   	#  rangex=0
	   	#  rangey=0
	   	#  rangez=0
           	# ranges_past = {}
                #broadcast the tag's transform from map -> tag (FRAME_ID)   
                br = tf.TransformBroadcaster()
                rospy.loginfo("pos['x'] is %s", pos['x'])
                rospy.loginfo("pos['y'] is %s", pos['y'])
                br.sendTransform((pos['x'], pos['y'], pos['z']),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),
                                FRAME_ID,
                                "map")
