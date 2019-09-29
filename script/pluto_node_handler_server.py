#!/usr/bin/env python

import os
import rospy
from pluto_node_handler.srv import *
from multiprocessing import Process
import threading
import subprocess
from time import sleep

service_dict ={	'websocket'	: '/usr/sbin/rosbridge-', 
				'minimal'  	: '/usr/sbin/pluto-minimal-', 
				'bringup'  	: '/usr/sbin/pluto-bringup-',
				'navigate'	: '/usr/sbin/pluto-navigate-',
				'slam'     	: '/usr/sbin/pluto-slam-'}

nodes_dict = {}  # a storage of nodes runing for every service, to populate at runtime
PAUSE_BTW_NODES_POPULATION = 15 # in sec, the thread delay to populate at runtime

def _lookup_service(service, type): 
	if service in service_dict:
		return service_dict[service] + type


def handle_start(req):
	rospy.loginfo("RCV handle_start [%s]", str(req.request))
	cmd = _lookup_service(req.request, 'start')
	if cmd is not None:
		def _cmd():
			print "handle_start sending command [%s]"%(cmd)
			os.system(cmd)
		p = Process(target=_cmd, )
		p.start()
		return PlutoServiceResponse("handle_start call " + cmd)
	else:
		return PlutoServiceResponse("handle_start failed call " + req.request)	


def handle_stop(req):
	rospy.loginfo("RCV handle_stop [%s]", str(req.request))
	cmd = _lookup_service(req.request, 'stop')
	if cmd is not None:
		os.system(cmd)
		return PlutoServiceResponse("handle_stop call " + cmd)
	else:
		return PlutoServiceResponse("handle_stop failed call " + req.request)
	
	return PlutoServiceResponse("handle_stop")
 
 
def _exec_cmd_remove_charachters(cmd):
	# it is necessary to run the environment to have some local variables with the linobot configuration (LINOLIDAR e.g.)
	result = subprocess.check_output(cmd,stderr=subprocess.STDOUT,shell=True)#, env=env_var)
	result = result.split('\n')
	return [_s.replace('/', '' ) for _s in result if len(_s) > 0]
 
 
def _populate_nodes(node_name, start_delayed=0):
	sleep(start_delayed) # adding a delay, starting all the thread simultaneuosly 
	
	service = node_name
	rospy.loginfo("_populate_nodes [%s]", str(node_name))

	if service in service_dict:
		# populate the first time the services
		if not service in nodes_dict:
			cmd = 'roslaunch --nodes plutobot '+service+'.launch' # returns the name of nodes running with this launch file		
			try:
				running_nodes = _exec_cmd_remove_charachters(cmd)
				nodes_dict[service] = running_nodes
				rospy.loginfo("For node [%s] found %s", str(node_name), str(running_nodes))
				return PlutoServiceResponse('service: ' + service + ' not in dict nodes_dict: ' + str(nodes_dict))
			except  subprocess.CalledProcessError as ex:  
				retval = 'Catched an exception executing '+str(cmd)+'\n' +str(ex) + '\noutput=' + str(ex.output)
				return PlutoServiceResponse(retval)
		else:
			pass
						

def handle_check(req):

	print "RCV handle_check [%s]"%(req.request)
	#  roslaunch --nodes plutobot bringup.launch
	service = req.request
	if service in service_dict:
		# populate the first time the services
		if not service in nodes_dict:
			return 'non available'
		else:
			running_nodes = nodes_dict[service]

		active_services = {}
		for _n in running_nodes:
			cmd = 'ps aux | grep __name:=' + _n # extract all running nodes
			try:
				results = _exec_cmd_remove_charachters(cmd)			
			except  subprocess.CalledProcessError as ex:  
				rospy.logwarn("handle_check exception in [%s]: %s - %s", str(_n) , str(ex), str(ex.output))
				retval = 'Catched an exception executing '+str(cmd)+'\n' +str(ex) + '\noutput=' + str(ex.output)
				return PlutoServiceResponse(retval)
			result = []
			for _r in results:
				if ('ps aux' in _r or 'grep' in _r): # there is always the grep items (possibly osberved more!)
					continue
				else:
					result.append(_r)
			
			if len(result)==0:
				active_services[_n] = 'stopped'
			elif len(result) == 1:
				active_services[_n] = 'active'
			else:	
				active_services[_n] = 'too much'
				
		active=too_much=stopped = 0
		
		retval = 'null'
		if len(active_services)	 == len(running_nodes):
			for k in active_services.keys():
				if active_services[k] == 'stopped':
					stopped =stopped + 1
				elif active_services[k] == 'active':
					active = active + 1
				elif   active_services[k] == 'too much':
					too_much = too_much +1
				# else: unknown??
			if too_much > 0:
				retval = 'internal_error'
			elif stopped==len(running_nodes):
				retval = 'stopped'
			elif active==len(running_nodes):
				retval = 'active'
			else:
				retval = 'inconsitent'
		else:
			retval = 'node_number_inconsitency'
		
		rospy.loginfo("handle_check retval for [%s]: %s", str(service), str(retval) )
		return PlutoServiceResponse(retval)
	else:
		rospy.logwarn("handle_check unknwon_service for [%s]: ", str(service) )
		return PlutoServiceResponse("unknwon_service " + service)
		

def usage():
    return "usage: %s str service [websocket|minimal|bringup|navigate|slam]) "%sys.argv[0]


def pluto_node_handler_server():
	# populate the service nodes_dict
	for _n,_k in  enumerate(service_dict.keys()):
		threading.Thread(target=_populate_nodes, args=(_k,_n*PAUSE_BTW_NODES_POPULATION)).start()
		
	# define the services
	rospy.init_node('pluto_node_handler_server') #from example online ('add_two_ints_server')
	s1 = rospy.Service('launch_pluto_node', PlutoService , handle_start) #s = rospy.Service('add_two_ints', String, handle_add_two_ints)
	s2 = rospy.Service('stop_pluto_node', PlutoService , handle_stop) #s = rospy.Service('add_two_ints', String, handle_add_two_ints)
	s3 = rospy.Service('check_pluto_node', PlutoService , handle_check) #s = rospy.Service('add_two_ints', String, handle_add_two_ints)
	
	print "Ready to run any challenging Pluto navigation."
	rospy.spin()

if __name__ == "__main__":
	pluto_node_handler_server()
