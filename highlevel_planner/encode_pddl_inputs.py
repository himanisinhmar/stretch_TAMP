#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  7 15:56:03 2022

@author: guy
"""

import json


max_mass_robot_can_carry = 2
user_defined_initial_conditions = None
user_defined_goal_conditions = '(and (has_liquid bowl) (not (is_robot_carrying)))'

def writeDomainFile(fid, objects):
    fid.write("(define (domain table_manipulation)\n")
    fid.write(";; Defining options for the planning system\n")
    fid.write("(:requirements :adl :typing :action-costs )\n")
    fid.write("	\n")
    fid.write(";; Defining types\n")
    fid.write("(:types\n")
    fid.write("  loc1 loc2 loc3 loc4 - virtobj\n")
    fid.write("  objects robot - physobj\n")
    fid.write(")\n")
    fid.write("	\n")
    fid.write(";; holding integer values stuff \n")
    fid.write("(:functions\n")
    fid.write("    (total-cost) ; the total cost of the operations taken\n")
    fid.write(")\n")
    fid.write("\n")
    fid.write("(:predicates\n")
    fid.write(" 	(gripped ?obj - objects) ; robot is gripping that object already?\n")
    fid.write(" 	(at ?obj - objects ?loc - virtobj)      ; treating the objects as locations as well\n")
    fid.write(" 	(can_carry ?obj)         ; whether the robot can pick it up (user-defined, mass/dimensions restrictions, etc...) \n")
    fid.write(" 	(is_robot_carrying)      ; whether the robot is carrying something \n")
    fid.write(" 	(man_disable ?obj1 ?obj2)  ; restrict some movement because it is not physically possible \n")

    list_of_affordances = []
    for obs,v in objects.items():
        if('affordances' in v):
            for affordance, properties in v['affordances'].items():
                affordance_name = ''.join([i for i in affordance if i.isalpha()])
                if(affordance_name not in list_of_affordances):
                    list_of_affordances.append(affordance_name)
                    fid.write(f" 	(has_{affordance_name} ?obs)  ; automatically generated for affordance {affordance_name}\n")
                    fid.write(f" 	(able_{affordance_name} ?obs ?loc - virtobj) ; automatically generated for affordance {affordance_name}\n")

    fid.write(")\n")
    fid.write("	\n")
    fid.write("; goto an object's location (IK is handled elsewhere in the low-level planner) and \n")
    fid.write("; it is also not in its previous place anymore\n")
    fid.write("(:action gofromto\n")
    fid.write(" :parameters ( ?obj_from - objects ?from_loc - virtobj ?obj_to - objects ?to_loc - virtobj )\n")
    fid.write(" :precondition\n")
    fid.write("	(and (not (at ?obj_to ?to_loc)) (at ?obj_from ?from_loc) (not (is_robot_carrying)) (not (man_disable ?obj_from ?obj_to)) )\n")
    fid.write(" :effect\n")
    fid.write("	(and (at ?obj_to ?to_loc) (not (at ?obj_from ?from_loc)) (increase (total-cost) 1) )\n")
    fid.write(")\n")
    fid.write("	\n")
    fid.write("; move an object (IK is handled elsewhere in the low-level planner) \n")
    fid.write("(:action move\n")
    fid.write(" :parameters ( ?obj_from - objects ?from_loc - virtobj  ?obj_to - objects ?to_loc - virtobj )\n")
    fid.write(" :precondition\n")
    fid.write("	(and (not (at ?obj_to ?to_loc)) (at ?obj_from ?from_loc) (gripped ?obj_from) (is_robot_carrying) (not (man_disable ?obj_from ?obj_to)) )\n")
    fid.write(" :effect\n")
    fid.write("	(and (at ?obj_to ?to_loc) (increase (total-cost) 1) )\n")
    fid.write(")\n")
    fid.write("	\n")
    fid.write("; grip action\n")
    fid.write("(:action grip\n")
    fid.write(" :parameters ( ?obj - objects ?loc - virtobj)\n")
    fid.write(" :precondition\n")
    fid.write("	(and (not (gripped ?obj)) (not (is_robot_carrying)) (at ?obj ?loc) (can_carry ?obj) )\n")
    fid.write(" :effect\n")
    fid.write("	(and ( gripped ?obj ) (is_robot_carrying) (increase (total-cost) 1) )\n")
    fid.write(")\n\n")
    fid.write("; ungrip action\n")
    fid.write("(:action ungrip\n")
    fid.write(" :parameters ( ?obj - objects )\n")
    fid.write(" :precondition\n")
    fid.write("	(gripped ?obj) \n")
    fid.write(" :effect\n")
    fid.write("	(and (not (gripped ?obj) ) (not (is_robot_carrying)) (increase (total-cost) 1) )\n")
    fid.write(")\n\n")

    list_of_affordances = []
    for obs,v in objects.items():
        if('affordances' in v):
            for affordance, properties in v['affordances'].items():
                affordance_name = ''.join([i for i in affordance if i.isalpha()])
                if(affordance_name not in list_of_affordances):
                    list_of_affordances.append(affordance_name)
                    fid.write(f"; automatically generated for affordance {affordance_name} \n")
                    fid.write(f"(:action do_{affordance_name}\n")
                    fid.write(" :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj)\n")
                    fid.write(" :precondition\n")
                    fid.write(f"	(and (at ?primary ?prime_loc) (gripped ?primary) (able_{affordance_name} ?primary ?prime_loc) \n")
                    fid.write(f"	     (at ?secondary ?sec_loc) (has_{affordance_name} ?secondary) )\n")
                    fid.write(" :effect\n")
                    fid.write(f"	(and (has_{affordance_name} ?primary) (increase (total-cost) 1) )\n")
                    fid.write(")\n\n")
                    fid.write(f"; automatically generated for negate affordance {affordance_name} \n")
                    fid.write(f"(:action cease_{affordance_name}\n")
                    fid.write(" :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj )\n")
                    fid.write(" :precondition\n")
                    fid.write(f"	(and (at ?primary ?prime_loc) (gripped ?primary) (able_{affordance_name} ?primary ?prime_loc) (has_{affordance_name} ?primary) \n")
                    fid.write(f"	     (at ?secondary ?sec_loc) (able_{affordance_name} ?secondary ?sec_loc) )\n")
                    fid.write(" :effect\n")
                    fid.write(f"	(and (has_{affordance_name} ?secondary) (not (has_{affordance_name} ?primary)) (increase (total-cost) 1) )\n")
                    fid.write(")\n\n")



    fid.write(")\n")

def writeProblemFile(fid, objects):
    fid.write("(define (problem tbl_task) \n")
    fid.write("(:domain table_manipulation)\n")
    fid.write("(:objects\n")
    fid.write("    ")
    for obj in objects:
        fid.write(f'{obj} ')
    fid.write("idle - objects\n")
    fid.write("    loc1 loc2 loc3 loc4 - virtobj\n")
    fid.write("	end_eff extension - robot\n")
    fid.write(")\n")
    fid.write("	\n")
    fid.write("(:init\n")
    if(user_defined_initial_conditions):
        for cond in user_defined_initial_conditions:
            fid.write(f"\t{cond} ; user defined initial conditions\n")
    fid.write("\t(at idle loc1) ; where the robot starts from\n")
    fid.write("\t; now, all the things that are autogenerated from the json affordances file")
    for obs,v in objects.items():
        fid.write("\n\t")
        list_of_affordances = []
        if( v['mass'] <= max_mass_robot_can_carry):
            fid.write(f"(can_carry {obs}) ")
        if('affordances' in v):
            for affordance, properties in v['affordances'].items():
                affordance_name = ''.join([i for i in affordance if i.isalpha()])
                loc = ''.join([i for i in affordance if i.isnumeric()])
                if(affordance_name not in list_of_affordances):
                    list_of_affordances.append(affordance_name)
                    if(properties['has'] == 1):
                        fid.write(f"(has_{affordance_name} {obs}) ")
                    if(properties['able'] == 1):
                        fid.write(f"(able_{affordance_name} {obs} loc{loc}) ")
                else:
                    fid.write(f"(able_{affordance_name} {obs} loc{loc}) ")
    fid.write("\n\t(= (total-cost) 0); set the objective cost to zero\n")
    fid.write(")\n")
    fid.write("\n")
    fid.write("(:goal \n")
    fid.write(f"		{user_defined_goal_conditions}\n")
    fid.write(")\n")
    fid.write("	\n")
    fid.write("; the objective is to minimize the costs\n")
    fid.write("(:metric minimize (total-cost))\n")
    fid.write("	\n")
    fid.write(")\n")


if __name__ == "__main__":
    with open('environment.json', 'r') as env_file:
        environment = json.load(env_file)
        objects = environment["objects"]
        with open('domain.pddl', 'w') as dmn_file:
            writeDomainFile(dmn_file, objects)
        with open('problem.pddl', 'w') as prblm_file:
            writeProblemFile(prblm_file, objects)

    print('done creating files. run \"./runit.sh\" to solve.')
























