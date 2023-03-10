(define (problem tbl_task) 
(:domain table_manipulation)
(:objects
    table pot bowl ladle cup stick idle - objects
    loc1 loc2 loc3 loc4 - virtobj
	end_eff extension - robot
)
	
(:init
	(at idle loc1) ; where the robot starts from
	; now, all the things that are autogenerated from the json affordances file
	
	(has_liquid pot) (able_liquid pot loc1) (able_liquid pot loc2) 
	(able_liquid bowl loc1) 
	(can_carry ladle) (able_liquid ladle loc1) (able_liquid ladle loc2) 
	(can_carry cup) (able_liquid cup loc1) 
	(can_carry stick) (able_push stick loc1) 
	(= (total-cost) 0); set the objective cost to zero
)

(:goal 
		(and (has_liquid bowl) (not (is_robot_carrying)))
)
	
; the objective is to minimize the costs
(:metric minimize (total-cost))
	
)
