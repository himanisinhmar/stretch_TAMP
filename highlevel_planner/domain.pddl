(define (domain table_manipulation)
;; Defining options for the planning system
(:requirements :adl :typing :action-costs )
	
;; Defining types
(:types
  loc1 loc2 loc3 loc4 - virtobj
  objects robot - physobj
)
	
;; holding integer values stuff 
(:functions
    (total-cost) ; the total cost of the operations taken
)

(:predicates
 	(gripped ?obj - objects) ; robot is gripping that object already?
 	(at ?obj - objects ?loc - virtobj)      ; treating the objects as locations as well
 	(can_carry ?obj)         ; whether the robot can pick it up (user-defined, mass/dimensions restrictions, etc...) 
 	(is_robot_carrying)      ; whether the robot is carrying something 
 	(man_disable ?obj1 ?obj2)  ; restrict some movement because it is not physically possible 
 	(has_liquid ?obs)  ; automatically generated for affordance liquid
 	(able_liquid ?obs ?loc - virtobj) ; automatically generated for affordance liquid
 	(has_push ?obs)  ; automatically generated for affordance push
 	(able_push ?obs ?loc - virtobj) ; automatically generated for affordance push
)
	
; goto an object's location (IK is handled elsewhere in the low-level planner) and 
; it is also not in its previous place anymore
(:action gofromto
 :parameters ( ?obj_from - objects ?from_loc - virtobj ?obj_to - objects ?to_loc - virtobj )
 :precondition
	(and (not (at ?obj_to ?to_loc)) (at ?obj_from ?from_loc) (not (is_robot_carrying)) (not (man_disable ?obj_from ?obj_to)) )
 :effect
	(and (at ?obj_to ?to_loc) (not (at ?obj_from ?from_loc)) (increase (total-cost) 1) )
)
	
; move an object (IK is handled elsewhere in the low-level planner) 
(:action move
 :parameters ( ?obj_from - objects ?from_loc - virtobj  ?obj_to - objects ?to_loc - virtobj )
 :precondition
	(and (not (at ?obj_to ?to_loc)) (at ?obj_from ?from_loc) (gripped ?obj_from) (is_robot_carrying) (not (man_disable ?obj_from ?obj_to)) )
 :effect
	(and (at ?obj_to ?to_loc) (increase (total-cost) 1) )
)
	
; grip action
(:action grip
 :parameters ( ?obj - objects ?loc - virtobj)
 :precondition
	(and (not (gripped ?obj)) (not (is_robot_carrying)) (at ?obj ?loc) (can_carry ?obj) )
 :effect
	(and ( gripped ?obj ) (is_robot_carrying) (increase (total-cost) 1) )
)

; ungrip action
(:action ungrip
 :parameters ( ?obj - objects )
 :precondition
	(gripped ?obj) 
 :effect
	(and (not (gripped ?obj) ) (not (is_robot_carrying)) (increase (total-cost) 1) )
)

; automatically generated for affordance liquid 
(:action do_liquid
 :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj)
 :precondition
	(and (at ?primary ?prime_loc) (gripped ?primary) (able_liquid ?primary ?prime_loc) 
	     (at ?secondary ?sec_loc) (has_liquid ?secondary) )
 :effect
	(and (has_liquid ?primary) (increase (total-cost) 1) )
)

; automatically generated for negate affordance liquid 
(:action cease_liquid
 :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj )
 :precondition
	(and (at ?primary ?prime_loc) (gripped ?primary) (able_liquid ?primary ?prime_loc) (has_liquid ?primary) 
	     (at ?secondary ?sec_loc) (able_liquid ?secondary ?sec_loc) )
 :effect
	(and (has_liquid ?secondary) (not (has_liquid ?primary)) (increase (total-cost) 1) )
)

; automatically generated for affordance push 
(:action do_push
 :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj)
 :precondition
	(and (at ?primary ?prime_loc) (gripped ?primary) (able_push ?primary ?prime_loc) 
	     (at ?secondary ?sec_loc) (has_push ?secondary) )
 :effect
	(and (has_push ?primary) (increase (total-cost) 1) )
)

; automatically generated for negate affordance push 
(:action cease_push
 :parameters ( ?primary - objects ?prime_loc - virtobj ?secondary - objects ?sec_loc - virtobj )
 :precondition
	(and (at ?primary ?prime_loc) (gripped ?primary) (able_push ?primary ?prime_loc) (has_push ?primary) 
	     (at ?secondary ?sec_loc) (able_push ?secondary ?sec_loc) )
 :effect
	(and (has_push ?secondary) (not (has_push ?primary)) (increase (total-cost) 1) )
)

)
