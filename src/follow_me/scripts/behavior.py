#!/usr/bin/env python
import owyl
from owyl import blackboard
import rospy
import numpy as np
import time
from utils import clamp, waitForBT,delay
from std_msgs.msg import String
from std_msgs.msg import Int32
from robot.msg import robot_cmd
from robot.msg import sonar
from robot.msg import compass
from geometry_msgs.msg import Point



class behavior:
    def __init__(self,bb):
        self.blackboard=bb#blackboard.Blackboard("behavior")
        ##rospy.loginfo (self.blackboard["stt"])
        self.main_tree=self.create_main_tree()
       
        self.Error2=5
        self.cameraSettleTolerance=50
        self.bodySettleTolerance=2
        
        # This two are used for searching ball when the ball is not found;  (search function)
        # self.prevPanLen holds the lenght of the panAngle list before searching is performed
        self.prevPanLen=1
        # self.currentpanLen holds the current lenght of the panAngle list after searching is performed
        # It the two are different we can infer that the ball has been spotted during searching and search was success
        self.currentPanLen=1

        
    def create_main_tree(self):
        '''
            creates the main tree for owyl behavior tree

            Args:
                self
                The tree looks like this:
                                                                            root
                                                                             |
                                                                     owyl.repeatAlways
                                                                             |
                                                                        owyl.sequence
                                                                             |  
                            ---------------------------------------------------------------------------------------------------------------------------------------------------------
                            |                                                                       |                          |                            |                       |
                    owyl.selector                                                           owyl.repeatUntilSucceed    self.isCameraTrackDone()   owyl.repeatUntilSucceed      self.isWalkForward()
                           |                                                                        |                                                       |
                -------------------------------------------------------------------------       self.cameraTrack()                                  self.isBodyTrack()
                |                  |                |                   |               |       
        self.checkIfBall() self.search85()   self.search60()  self.search35() self.turnBody()


            Returns:
                Creates a tree and returns
                owyl.visit method to visit next nodes in the tree.
        '''
        # tree=owyl.repeatAlways(self.cameraTrack())  # For testing only the camera tracker ... see method cameraTrack() for details
        tree=owyl.repeatAlways(
                owyl.sequence(
                    owyl.selector(self.checkIfBall(),owyl.selector(self.search85(),self.search60(),self.search35(),self.turnBody()))
                    ,owyl.repeatUntilSucceed(self.cameraTrack())
                    ,self.isCameraTrackDone()
                    ,owyl.repeatUntilSucceed(self.isBodyTrack())
                    ,self.isWalkForward()
                    )
                )
         
        return owyl.visit(tree,blackboard=self.blackboard)
    
    def clamp(self,low, value, high):
        '''
            clips a value between low and high

            Args:
                low (lower margin), value (value passed), and high(upper marging)

            Returns:
                A clips version of value (float) that is assereted between low and high
        '''
        return max(min(high,value),low)

    def waitForBT(self):
        '''
            Delay code execution for 6 seconds > until bluetooth connection is secured.
        '''
        print "Waiting for bluetooth to connect..."
        time.sleep(6)
        print "Connected"

    def delay(self,t_sec):
        '''
            A time gap in seconds
            Args:
                t_sec > time of delay in 2-seconds
            Returns:
                None
        '''
        time.sleep(t_sec)    

    @owyl.taskmethod
    def checkIfBall(self,**kwargs):
        '''
            N.B. This is a python generator, not normal function

            Checks if the ball is already in the line of sight of the robot. saves a great deal of time that would be 
            wasted on searching.
            
            It waits for 2-seconds, then decides by checking if the call back function has already added pan angles
            to pan angles list.

            Args:
                self.blackboard["angles"]["pan"]

            Returns:
                Yields True : if the pan angles are greater than some constant values: meaning a ball is already spotted.
                Yields False: otherwise

            Parameters for optimization:
                1. The constant term that defines how many pan angles there should exist to decide the ball is already spotted
                    Larger values => resistant to noise but may result to failure to detect the ball.

                2. The delay time > before checking the number of pan angles.
                    Larger value > enough time to make sure a ball is not missed right under the nose.
                    Smaller value > good for speed.    
        '''

        # #rospy.loginfo("checking if ball exists")
        delay(2)
        if len(self.blackboard["angles"]["pan"])>30:
            # ball is already detected. Jump to camera mode
            # self.CT()
            self.reintialize()
            yield True
        else:
            self.reintialize()
            yield False
    @owyl.taskmethod
    def search85(self,**kwargs):
        '''
            N.B. This is a pyton generator, not normal function
            It searchs the ball from left to right at a tilt angle of 85 degrees.  (zero degree is when the camera 
                straight forward)
            
            Args:
                self.scanning_tilt => tilt angle for search

            Returns:
                Yields True : After breaking out of for loop that turns the pan from left to right. Meaning
                            a ball was spotted during search.

                Yields False: If it successfully searched over all range of pan angles 0-90 without spotting the ball.
        
            Parameters for Optimization:
                1. You can easily change the tilt search angle by chaning the variable self.
                2. By changing the constant added term when checking whether the ball is spotted or not; you can
                    change when the robot would consider a ball is spotted. Higher value means large noise reduction
                    but more delay time (or failure to spot the ball)
        '''
        # #rospy.loginfo("at 80 search")
        self.scanning_tilt=85            # Here you can change the tilt angle
        self.pan_head(0)
        delay(1)
        self.tilt_head(self.scanning_tilt)
        delay(1)
        # Next turn the body smoothly from right to left at a step of 5 degrees for a total of 90 degrees
        for i in range (10,95,10):
            self.pan_head(i)
            delay(0.5)
            # check if a ball was spotted every multiple of 10 degree angle
            if i%10==0:
                self.currentPanLen=len(self.blackboard["angles"]["pan"])
                # #rospy.loginfo("current  %f ,  prev  %f",self.currentPanLen,self.prevPanLen)
                if self.currentPanLen > self.prevPanLen+10:  # Here you can change the constant for optimization
                    # a ball was spotted (pan angle has been appended by the call back function)
                    # so break the loop and yield True
                    break
        
        if i <= 85:
            # #rospy.loginfo("got it")
            # #rospy.loginfo("before current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            self.blackboard["angles"]["pan"]=[i]
            self.blackboard["angles"]["tilt"]=[self.scanning_tilt]
            # #rospy.loginfo("After current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            yield True
        else:

            self.reintialize()
            self.blackboard["angles"]["pan"]=[90]
            yield False

        
    @owyl.taskmethod
    def search60(self,**kwargs):
        '''
            N.B. This is a pyton generator, not normal function
            It searchs the ball from left to right at a tilt angle of 60 degrees.  (zero degree is when the camera 
                straight forward)
            
            Args:
                self.scanning_tilt => tilt angle for search

            Returns:
                Yields True : After breaking out of for loop that turns the pan from left to right. Meaning
                            a ball was spotted during search.

                Yields False: If it successfully searched over all range of pan angles 0-90 without spotting the ball.
        
            Parameters for Optimization:
                1.You can easily change the tilt search angle by chaning the variable self.scanning_tilt
                2. By changing the constant added term when checking whether the ball is spotted or not; you can
                    change when the robot would consider a ball is spotted. Higher value means large noise reduction
                    but more delay time (or failure to spot the ball)
        '''
        # #rospy.loginfo("at 60 search")
        self.scanning_tilt=60
        # self.pan_head(0);
        delay(1)
        self.tilt_head(self.scanning_tilt);
        delay(1)
        # Next turn the body smoothly from right to left at a step of 5 degrees for a total of 90 degrees
        ii=range(0,95,10)
        ii.reverse()
        for i in ii:
            self.pan_head(i)
            delay(0.5)
            # check if a ball was spotted every 10 multiple degree angle
            if i%10==0:
                self.currentPanLen=len(self.blackboard["angles"]["pan"])
                # #rospy.loginfo("current  %f ,  prev  %f",self.currentPanLen,self.prevPanLen)
                if self.currentPanLen > self.prevPanLen+10:
                    # a ball was spotted (pan angle has been appended by the call back function)
                    # so break the loop and yield True
                    break
        
        if i >=5:
            # #rospy.loginfo("got it")
            # #rospy.loginfo("before current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            self.blackboard["angles"]["pan"]=[i]
            self.blackboard["angles"]["tilt"]=[self.scanning_tilt]
            # #rospy.loginfo("After current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            yield True
        else:
            self.reintialize()
            yield False


    @owyl.taskmethod
    def search35(self,**kwargs):
        '''
            N.B. This is a pyton generator, not normal function
            It searchs the ball from left to right at a tilt angle of 35 degrees.  (zero degree is when the camera 
                straight forward)
            
            Args:
                self.scanning_tilt => tilt angle for search

            Returns:
                Yields True : After breaking out of for loop that turns the pan from left to right. Meaning
                            a ball was spotted during search.

                Yields False: If it successfully searched over all range of pan angles 0-90 without spotting the ball.
        
            Parameters for Optimization:
                1. You can easily change the tilt search angle by chaning the variable self.scanning_tilt
                2. By changing the constant added term when checking whether the ball is spotted or not; you can
                    change when the robot would consider a ball is spotted. Higher value means large noise reduction
                    but more delay time (or failure to spot the ball)
        '''
        # #rospy.loginfo("at 35 search")
        self.scanning_tilt=35
        self.pan_head(0);
        delay(1)
        self.tilt_head(self.scanning_tilt);
        delay(1)
        # Next turn the body smoothly from right to left at a step of 5 degrees for a total of 90 degrees
        for i in range (10,95,10):
            self.pan_head(i)
            delay(0.5)
            # check if a ball was spotted every 10 multiple degree angle
            if i%10==0:
                self.currentPanLen=len(self.blackboard["angles"]["pan"])
                # #rospy.loginfo("current  %f ,  prev  %f",self.currentPanLen,self.prevPanLen)
                if self.currentPanLen > self.prevPanLen+10:
                    # a ball was spotted (pan angle has been appended by the call back function)
                    # so break the loop and yield True
                    break
        
        if i <= 85:
            # #rospy.loginfo("got it")
            # #rospy.loginfo("before current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            self.blackboard["angles"]["pan"]=[i]
            self.blackboard["angles"]["tilt"]=[self.scanning_tilt]
            # #rospy.loginfo("After current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
            yield True
        else:
            self.reintialize()
            yield False

    @owyl.taskmethod
    def turnBody(self,**kwargs):
        '''
            N.B. This is a python generator, not a normal function
            owyl leaf node

            Turns the body to the left three times to cover unsearched areas

            Args:
                self.blackboard["robot_cmd_pub"] => publisher to ROS (robot commands to order the robot)

            Returns:
                Yields True: 1. If the ball was spotted while in turning the body (breaked out of loop)
                Yields False: If it turned the body three times successfully
        '''
        #rospy.loginfo("at turn body search")
        delay(1)
        self.CT()
        for i in range(3):
            self.blackboard["robot_cmd_pub"].publish('turn left',200)
            if len(self.blackboard["angles"]["pan"])>10:
                break
            delay(4)
        if i<2:
            yield True
        else:
            self.reintialize()
            yield False

    @owyl.taskmethod
    def cameraTrack(self,**kwargs):
        '''
            N.B. this is a python generator, not normal function

            owyl leaf node that performs the camera tracking

            Args:
                self.blackboard, specifically:
                    self.blackboard["angles"]=> a dictionary that contains a list of pan and tilt angles of the camera.
                            self.blackboard["angles"]["pan"] and self.blackboard["angles"]["tilt"]
                    self.cameraSettleTolerance => number of pan angles required before any decision is done on ball settling.

                    1. checks if there are enough data to judge the ball is at rest in some place (requires that 
                        self.cameraSettleTolerance amount data be present in the list of pan angles).
                    
                    2. check if the ball is settled. (checks this by checking if the pan angle has settled.
                            cheks if the average of the last 10 pan angle readings is in acceptable tolerance to the last
                            pan angle reading -> uses self.Error2)
                    3. raises a sanity flag self.blackboard["ctSucceeded"] based on whether it tracked the ball successfully
                        (True) or it lost the ball during in the middle of trackig (False).

            Returns:
                Yields True : 1. If it tracked the ball successfully or 2. if it lost the ball during tracking. Differentiates
                            the two cases the the sanity flag explained in Args section #3. this is used by isCameraTrackDone()
                            leaf node to decide what to do next.
                Yields False: 1. If there is no enough data to do make any decision: so conitinue tracking
                              2. There is enough data but the ball hasn't settled yet.

            Parameters for optimization:
                The following parameters can be changed to optimize the camera tracking
                   1. self.cameraSettleTolrance => increases or decreases the amount of pan angles before decision
                      on ball settlement is made
                   2. self.Error2 => it is the tolerance of the average of the 10 pan angle readings to decide settlement.
                            Increase this and the will think the ball is settled immediately.
        '''
        rospy.loginfo("I'm in camera track mode")
        self.CTDone=False
        self.PrevPanData=len(self.blackboard["angles"]["pan"])
        self.counter=0
        while not self.CTDone:
            #rospy.loginfo("checking out the while loop")
            self.currPanData=len(self.blackboard["angles"]["pan"])
            if self.PrevPanData==self.currPanData:
                self.counter+=1

            if self.counter>=15:   # change this to find appropriate values
                

                # camera Track failed.... The ball is lost during tracking. so rather than getting stucked here break and
                # look for the ball.
                # Set the Camera track status flag => ctSucceeded to False. (failure)
                self.blackboard["ctSucceeded"]=False
                # Yield True to exit this leaf 'cause these node is called under owyl.repeatUntilSucceed.
                # Will notify next node that camera track has failed using the ctSucceed flag.
                #rospy.loginfo('about to exit camera track due to failure')
                self.CTDone=True
                yield True
            else:    
                if len(self.blackboard["angles"]["pan"])<self.cameraSettleTolerance:
                    # not enough data yet to decide wheter camera has settled or not!
                    ##rospy.loginfo("current pan angles %f  ",self.blackboard["angles"]["pan"][-1])
                    self.CT()
                    
                    self.PrevPanData=self.currPanData
                    yield False
                else:
                    ##rospy.loginfo("In else stat")
                    self.Mean=np.mean(self.blackboard["angles"]["pan"][-10:-1])
                    # #rospy.loginfo("This is the mean" + str(Mean))
                    self.check1=(self.blackboard["angles"]["pan"][-1]) + self.Error2
                    self.check2=(self.blackboard["angles"]["pan"][-1]) - self.Error2
                    # check if pan angles has settled (in effect camera track has settled)
                    self.PrevPanData=self.currPanData
                    if self.Mean<=self.check1 and self.Mean>=self.check2:
                        #record heading degrees right after the end of camera tracking
                        self.blackboard["bodyPose"].append(self.blackboard["angles"]["pan"][-1]-45)
                        self.blackboard["HeadingAfterCT"].append(self.getCompass()[-1])
                        #Camera Track done!
                        #rospy.loginfo("Done with CT")
                        self.blackboard["ctSucceeded"]=True
                        self.CTDone=True
                        yield True
                    else:
                        self.CT()
                        yield False
            
        

        '''
        ##  Testing code for camera tracker!
            # Comment the above code; uncomment this one and paste the below code on the build_tree to test the camera
            # tracking node only. Can be used to find a good PD controller gain just by running the camera tracker only.
            #   "paste this code to the creat_main_tree at the top of the class"  
            #             owyl.repeatAlways(cameraTrack())  OR just uncomment it in that function
        # while True:
        #     self.CT()
        #     yield False
        '''


    @owyl.taskmethod
    def isCameraTrackDone(self, **kwargs):
        # #rospy.loginfo("I'm called to after camear track")
        '''
            N.B. This is generator, not a normal python function
            Checks if the camera track is done successfully or not.
            
            Another code can be inserted here; like determining to what direction the ball was lost and issue the
            robot to search in that direction rather than to make "a stupid search"

            Args:
                self.blackboard; specifically:
                    self.blackboard["ctSucceeded"] => w/c is a flag set by the CameraTrack() leaf.

            Returns:
                Yields False if cameraTrack was not successful because the ball was lost during tracking
                Yields True if cameraTrack was a success.

        '''
        if self.blackboard["ctSucceeded"]==True:
            yield True
        else:
            yield False


    @owyl.taskmethod
    def isBodyTrack(self, **kwargs):
        '''
            N.B. This is a python generator, not a normal function
            Owyl leaf Node
            This leaf performs the body tracking of robosapien
            It's sole purpose is to align the body of robosapien in the direction of the ball.
            It uses the compass reading to achieve its goal

            Args:
                self.blackboard  specifically:
                    self.blackboard["HeadingAfterCT"] => w/c is the heading right after Camera Track is done
                    self.blackboard["compass_data"]=> w/c a dictionary that hold a list of compass readings
                    self.blackboard["bodyPose"]=> w/c is a list that is used to hold the by how much the pan angle 
                        (right after Camera Track is done) deviates from the initial pan position (i.e 45 degree).
                         We can know how much the body must turn to align itself in the direction of the ball.)

            returns:
                Yields True as soon as the body is aligned in the direction of the ball
                Yields False if the body needs alignement to the ball yet.

        '''


        # #rospy.loginfo("I'm in Body Track mode")
        bodyPose=self.blackboard["bodyPose"][0]  # This holds how much the body needs to move. Check this value 
                                                 # track is not satisfactory. Possible problems are because 45 is 
                                                # used as a base initial value. so make sure 45 degree pan angle turns the camera
                                                #to a straight forward view. Otherwise change the code in the camera track mode.
        heading=self.getCompass()
        self.BTDone=False
        
        while not self.BTDone:
            drift=heading[-1]-self.getHeadingCT()

            # #rospy.loginfo("bodyPose= %f,  CH= %f, ref= %f, drift= %f",bodyPose,heading[-1],self.getHeadingCT(),drift)
            if abs(drift)<abs(bodyPose):
                # Body has to got rotate some degree yet
                 
                if bodyPose >=0:
                    direction='turn left'
                else:
                    direction='turn right'
                self.blackboard["robot_cmd_pub"].publish(direction,100)
                delay(2.0)
                yield False
                # #rospy.loginfo("working on turning on the body")
            else:
                # if the drift of compass is in range,  Done with BodyTrack
                # #rospy.loginfo("Drift is correct")
                delay(2.0)
                self.BTDone=True
                yield True
    @owyl.taskmethod
    def isWalkForward(self, **kwargs):
        '''
            Owyl leaf node
            Commands the robot to move forward. (no complicated stuff; Just publishes to the robot_cmd_pub)

            Args:
              (self), and kwargs (a dictionary that hold the blackboard)


            Returns:
                Yields True right after  a 3second delay of issueing the walk command.
        '''
        self.pan_head(self.blackboard["start_pan"])
        delay(0.1)
        self.tilt_head(self.blackboard["start_tilt"])
        delay(0.1)
        self.blackboard["robot_cmd_pub"].publish('walk forward',250)
        delay(3.0)
        self.reintialize()
        yield True


    # return all values to default to restart the whole process of tracking again   
    def reintialize(self):
        '''
            sets most of the blackboard values to default values.

            Args:
                (self)=> blackboard dictionary.

            returns:
                None

        '''

        self.blackboard["angles"] = {"pan" :[self.blackboard["start_pan"]], "tilt" :[self.blackboard["start_tilt"]]}
        self.blackboard["sonar_data"] = {"distance" : [200]}
        self.blackboard["compass_data"]={"heading":[]}
        self.blackboard["bodyPose"]=[]
        self.blackboard["HeadingAfterCT"]=[]
        self.prevPanLen=1
        self.currentPanLen=1


    def CT(self):
        '''
            This function publishes to the camera servo motors publishers  i.e /act/robot/set_pan_angle and /act/robot/set_tilt_angle
            Publishes the last pan and tilt angles from the pan and tilt lists
        '''

        ##rospy.loginfo("CALLED")
        # #rospy.loginfo("pan %f  , tilt %f  ",self.blackboard["angles"]["pan"][-1],self.blackboard["angles"]["tilt"][-1])
        self.tilt_head(self.blackboard["angles"]["tilt"][-1])
        delay(0.1)
        self.pan_head(self.blackboard["angles"]["pan"][-1])
        delay(0.1)


    def pan_head(self,angle):    
        '''
            A publisher function to the pan servo

            arguments (angle in degree. Should be  0-90, otherwise it will be clipped to 0 or 90)
        '''
        if not rospy.is_shutdown():
            self.blackboard["pan_angle_pub"].publish(angle)

    def tilt_head(self,angle):
        '''
            A publisher function to the tilt servo motors

            arguments (angle in degree. should be 0-90, otherwise it will be clippped to 0 or 90)
        '''
        if not rospy.is_shutdown():
            self.blackboard["tilt_angle_pub"].publish(angle) 

    def stop_movement():
        '''
           A publihser function to the robot.  It will stop any command send to the robot.   
              publishes to /act/robot/send_move_command

        '''
        if not rospy.is_shutdown():
            self.blackboard["robot_cmd_pub"].publish('stop', 0)

    
    def getSonarDistance(self):
        '''
           Function to access the list of sonar readings
        '''
        return self.blackboard["sonar_data"]["distance"]


    def getCompass(self):
        '''
           Function to access the list of compass reading
        '''
        return self.blackboard["compass_data"]["heading"]

    def getHeadingCT(self):
        '''
            Function to access the first reading of the compass right after the camera has settled on tracking the object
            This heading value is used to calculate (with the help of the current camera pan angle) how much the body needs
            turn.

            Used by the bodyTrack node of the owyl behavior tree.


            returns :  the compass reading right after the camera track has settled
        '''
        return self.blackboard["HeadingAfterCT"][0]




def faceDetected(points):
    '''
    This is the callback function that is called when ever a ball is detected!
    It performs the PD controller task required to track the ball
    

    Inputs =>  geometry_msgs.points==>  points is a pair of (x,y) that is the top_left corner of the tracked ball
    
    output:
        It directly writes to the angles dictionary that is a dictionary of list of pan and tilt angles indepedently
        It Performs PD controller on the pan and tilt command then append it to the dictionary so that any function 
        interested can use them to publish the values.

    '''

    global tPrev
    tCurrent=time.time()
    dt=abs(tCurrent-tPrev)
    # grab the top left corner as x-axis
    x=points.x
    # gradthe top left corner as y-axis
    y=points.y

    # Ideally the point should be at the center of the camera view (i.e (x,y)= (0.5,0.5))
    ideal_left = 0.3
    ideal_top  = 0.45 
    
    # Find the derivative terms
    dx=(x-x_prev)/dt
    dy=(y-y_prev)/dt


    # Find the control signal using PD controller
    correction_pan = kx*(ideal_left - x)-kdx*(dx) 
    correction_tilt= ky*(ideal_top - y)-kdy*(dy)

    # prepare for clamping the values
    temp1=board["angles"]["pan"][-1] + correction_pan
    temp2=board["angles"]["tilt"][-1]- correction_tilt

    # #rospy.loginfo("This is pan and tilt correction %f   %f :  ",correction_pan, correction_tilt)
    # Clamp pan and tilt angles
    board["angles"]["pan"].append(clamp(0, temp1, 90))
    board["angles"]["tilt"].append(clamp(0, temp2, 90))
    tPrev=tCurrent

def readSonarDist(data):
    '''
        A call back function that reads and appends the sonar distance

        Argus:
            data =>  contains data.distance_cm > sonar cm reading

        returns:
            None > just appends the reading to the dictionary board["sonar_data"]["distance"]
    '''
    board["sonar_data"]["distance"].append(data.distance_cm)

def readCompass(data):
    '''
    A call back function that reads and appends the compass reading in degree

    Argus:
        data =>  contains data.heading_deg > Digital compass reading in degree (reference Absolute North)

    returns:
        None > just appends the reading to the dictionary board["compass_data"]["heading"]
    '''
    board["compass_data"]["heading"].append(data.heading_deg)



if __name__=="__main__":
    # Initialize Node
    rospy.init_node('behavior', anonymous = False)
    
    
    # Initialize Publishers
    pan_angle_pub = rospy.Publisher('/act/robot/set_pan_angle', Int32, queue_size = 1)
    tilt_angle_pub = rospy.Publisher('/act/robot/set_tilt_angle', Int32, queue_size = 1)
    robot_cmd_pub = rospy.Publisher('/act/robot/send_move_command', robot_cmd, queue_size = 1)
    
    # Initialize subscribers
    rospy.Subscriber('/ball_pose',Point, faceDetected)
    rospy.Subscriber('/sense/robot/get_sonar_cm', sonar, readSonarDist)
    rospy.Subscriber('/sense/robot/get_compass_deg',compass,readCompass)
    


    # create an object of black board 
    board=blackboard.Blackboard()
    
    # publishers on board
    board["pan_angle_pub"]  = pan_angle_pub
    board["tilt_angle_pub"] = tilt_angle_pub
    board["robot_cmd_pub"]  = robot_cmd_pub
    
    # Init variables
    board["angles"] = {"pan" :[45], "tilt" :[60]}
    board["sonar_data"] = {"distance" : [200]}
    board["compass_data"]={"heading":[0]}
    board["bodyPose"]=[]
    board["start_pan"]=45
    board["start_tilt"]=60
    board["HeadingAfterCT"]=[] # This records the heading right after the camera track has stopped
    board["ctSucceeded"]=True  # Flag to determine whether camera track was a success of failure. True by default (Success)

    
    # Initialize terms for PD controllers
    x_prev=board["start_pan"]   # previous x-pos
    y_prev=board["start_tilt"]   # previous y-Pos
    kx=4.5       # constant term for x direction
    ky=2.5       # constant term for y direction
    kdx=0.0008      # differential pan gain 
    kdy=-0.0002    # differential tilt gain
    dt=0.1     # rate of time between call back ... I guessed this one
    tPrev=time.time()
    # Create a behavior tree using the blackboard object board
    be=behavior(board)

    # call to the main tree (root)
    be_tree=be.main_tree
    
    # Wait for bluetooth to connect
    waitForBT()

    # Move head to default position
    be.pan_head(board["start_pan"])
    be.tilt_head(board["start_tilt"])
        
    rate=rospy.Rate(100)#100hz
    # rospy.spin()
    while not rospy.is_shutdown():
        ##rospy.loginfo("Here in the loop")
        be_tree.next()
        rate.sleep()
    ##rospy.loginfo("bye")