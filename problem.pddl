(define (problem padm-project)
  (:domain padm-kitchen)
  (:objects sugar_box0  potted_meat_can1  roboArm  indigo_drawer_top )
  (:init (onTable potted_meat_can1) (onBurner sugar_box0) (clearRoboArm) (open indigo_drawer_top))
  (:goal (and (ontable sugar_box0) (inside indigo_drawer_top potted_meat_can1)))
)



