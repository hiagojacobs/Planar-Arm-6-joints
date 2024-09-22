import numpy as np

MAX_STEP = 10

class JacobianController:
    def __init__(self, arm):
        # the controller should save a reference to the arm it's controlling
        self.arm = arm

    def control(self, target):
        if self.arm.get_num_joints() == 6:
            # if arm has 2 joints
            self.control6J2D(target)
        elif self.arm.get_num_joints() == 3:
            # if arm has 3 joints
            self.control3J2D(target)
        else:
            raise Exception("JacobianController.control(target): Can't control an arm with this amount joints.")
            
        a = self.arm.get_num_joints()             
        print(a)

    def control6J2D(self, target):
        # the control method receives a target
        
############################################### 6articulaciones ####################################################################
        
        curr_end = self.arm.endeffector()       #cinematica direta implementado
        
        delta_C = target - curr_end            #variación cartesiana
        
        if (delta_C.r > MAX_STEP):
        	delta_C.r = MAX_STEP
        	
        theta1 = self.arm.thetas[0]
        theta2 = self.arm.thetas[1]
        theta3 = self.arm.thetas[1]
        theta4 = self.arm.thetas[1]
        theta5 = self.arm.thetas[1]
        theta6 = self.arm.thetas[1]
        
        l1 = self.arm.lengths[0]
        l2 = self.arm.lengths[1]
        l3 = self.arm.lengths[1]
        l4 = self.arm.lengths[1]
        l5 = self.arm.lengths[1]
        l6 = self.arm.lengths[1]
        
        a00 = -l1*np.sin(theta1) - l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1) - l3*np.sin(theta3)*np.cos(theta1 + theta2) - l3*np.sin(theta1 + theta2)*np.cos(theta3) - l4*np.sin(theta4)*np.cos(theta1 + theta2 + theta3) - l4*np.sin(theta1 + theta2 + theta3)*np.cos(theta4) - l5*np.sin(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l5*np.sin(theta1 + theta2 +     theta3 + theta4)*np.cos(theta5) - l6*np.sin(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5) - l6*np.sin(theta1 + theta2 	+ theta3 + theta4 + theta5)*np.cos(theta6)
        a01 = -l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1) - l3*np.sin(theta3)*np.cos(theta1 + theta2) - l3*np.sin(theta1 + theta2)*np.cos(theta3) - l4*np.sin(theta4)*np.cos(theta1 + theta2 + theta3) - l4*np.sin(theta1 + theta2 + theta3)*np.cos(theta4) - l5*np.sin(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l5*np.sin(theta1 + theta2 + theta3 + theta4)*np.cos(theta5) - l6*np.sin(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5) - l6*np.sin(theta1 + theta2 + theta3 + theta4 + theta5)*np.cos(theta6)
        a02 = -l3*np.sin(theta3)*np.cos(theta1 + theta2) - l3*np.sin(theta1 + theta2)*np.cos(theta3) - l4*np.sin(theta4)*np.cos(theta1 + theta2 + theta3) - l4*np.sin(theta1 + theta2 + theta3)*np.cos(theta4) - l5*np.sin(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l5*np.sin(theta1 + theta2 + theta3 + theta4)*np.cos(theta5) - l6*np.sin(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5) - l6*np.sin(theta1 + theta2 + theta3 + theta4 + theta5)*np.cos(theta6)
        a03 = -l4*np.sin(theta4)*np.cos(theta1 + theta2 + theta3) - l4*np.sin(theta1 + theta2 + theta3)*np.cos(theta4) - l5*np.sin(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l5*np.sin(theta1 + theta2 + theta3 + theta4)*np.cos(theta5) - l6*np.sin(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5) - l6*np.sin(theta1 + theta2 + theta3 + theta4 + theta5)*np.cos(theta6)
        a04 = -l5*np.sin(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l5*np.sin(theta1 + theta2 + theta3 + theta4)*np.cos(theta5) - l6*np.sin(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5) - l6*np.sin(theta1 + theta2 + theta3 + theta4 + theta5)*np.cos(theta6) 
        a05 = l1*np.cos(theta1) - l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2) - l3*np.sin(theta3)*np.sin(theta1 + theta2) + l3*np.cos(theta3)*np.cos(theta1 + theta2) - l4*np.sin(theta4)*np.sin(theta1 + theta2 + theta3) + l4*np.cos(theta4)*np.cos(theta1 + theta2 + theta3) - l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a10 = l1*np.cos(theta1) - l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2) - l3*np.sin(theta3)*np.sin(theta1 + theta2) + l3*np.cos(theta3)*np.cos(theta1 + theta2) - l4*np.sin(theta4)*np.sin(theta1 + theta2 + theta3) + l4*np.cos(theta4)*np.cos(theta1 + theta2 + theta3) - l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a11 = -l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2) - l3*np.sin(theta3)*np.sin(theta1 + theta2) + l3*np.cos(theta3)*np.cos(theta1 + theta2) - l4*np.sin(theta4)*np.sin(theta1 + theta2 + theta3) + l4*np.cos(theta4)*np.cos(theta1 + theta2 + theta3) - l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a12 = -l3*np.sin(theta3)*np.sin(theta1 + theta2) + l3*np.cos(theta3)*np.cos(theta1 + theta2) - l4*np.sin(theta4)*np.sin(theta1 + theta2 + theta3) + l4*np.cos(theta4)*np.cos(theta1 + theta2 + theta3) - l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a13 = -l4*np.sin(theta4)*np.sin(theta1 + theta2 + theta3) + l4*np.cos(theta4)*np.cos(theta1 + theta2 + theta3) - l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a14 = -l5*np.sin(theta5)*np.sin(theta1 + theta2 + theta3 + theta4) + l5*np.cos(theta5)*np.cos(theta1 + theta2 + theta3 + theta4) - l6*np.sin(theta6)*np.sin(theta1 + theta2 + theta3 + theta4 + theta5) + l6*np.cos(theta6)*np.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        a15 = l6*np.cos(theta1 + theta2 + theta3 + theta4 + theta5 + theta6)

	
        J = np.array( [[a00, a01, a02, a03, a04, a05],         #calculando jacobiana
                      [a10, a11, a12, a13, a14, a15]] )
	
        J_inv = np.linalg.pinv(J)          #inversa de Jacobiana
 
        delta_theta = J_inv.dot(np.array([delta_C.x, delta_C.y]))  #variación de theta
 
        self.arm.move(delta_theta)
        
############################################ 2articulaciones(funciona) ############################################
        
        #curr_end = self.arm.endeffector()       #cinematica direta implementado
        
        #delta_C = target - curr_end            #variación cartesiana
        
        #if (delta_C.r > MAX_STEP):
        #	delta_C.r = MAX_STEP
        
        #theta1 = self.arm.thetas[0]
        #theta2 = self.arm.thetas[0]
        
        #l1 = self.arm.lengths[0]
        #l2 = self.arm.lengths[1]
        
        #a00 = -l1*np.sin(theta1) - l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1)
        #a01 = -l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1)
        #a10 = l1*np.cos(theta1) - l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2)
        #a11 = -l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2)
 
	
        #J = np.array( [[a00, a01],         #calculando jacobiana
        #              [a10, a11]] )
	
        #J_inv = np.linalg.pinv(J)          #inversa de Jacobiana
 
        #delta_theta = J_inv.dot(np.array([delta_C.x, delta_C.y]))  #variación de theta
 
        #self.arm.move(delta_theta)
        
    def control3J2D(self, target):
        # the control method receives a target
        pass
