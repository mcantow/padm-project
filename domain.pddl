(define (domain padm-kitchen)
  (:requirements :strips :negative-preconditions)
  (:predicates (onTable ?x ) (onBurner ?x ) (inside ?x ?y ) (closed ?x) (holding ?x ) (clearRoboArm))
  (:action pickupFromDrawer
    :parameters (?ob ?drawer )
    :precondition (and (clearRoboArm) (not(closed ?drawer)) (inside ?drawer ?ob) )
    :effect (and (holding ?ob) (not (clearRoboArm)) (not (inside ?drawer ?ob)))
  )
  
  (:action pickupFromTable
    :parameters (?ob   )
    :precondition (and (clearRoboArm) (onTable ?ob) )
    :effect (and (holding ?ob) (not (clearRoboArm)) (not (onTable ?ob)))
  )
  
  (:action pickupFromBurner
    :parameters (?ob )
    :precondition (and (clearRoboArm) (onBurner ?ob) )
    :effect (and (holding ?ob) (not (clearRoboArm)) (not (onBurner ?ob)))
  )

  (:action grabDrawer
    :parameters (?drawer )
    :precondition (clearRoboArm)
    :effect (and (holding ?drawer) (not (clearRoboArm)))
  )
  
  (:action openDrawer
    :parameters (?drawer)
    :precondition (and  (holding ?drawer) (closed ?drawer) )
    :effect (and (not(closed ?drawer)) (not(holding ?drawer)) (clearRoboArm))
  )
  
  (:action putOnTable
    :parameters (?ob )
    :precondition (holding ?ob)
    :effect (and (clearRoboArm) (onTable ?ob) (not (holding ?ob)))
  )
  (:action putInDrawer
    :parameters (?ob ?drawer)
    :precondition (and (holding ?ob) (not(closed ?drawer)) )
    :effect (and (clearRoboArm) (inside ?drawer ?ob) (not (holding ?ob)))
  )
)
