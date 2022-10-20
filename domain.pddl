(define (domain padm-kitchen)
  (:requirements :strips :negative-preconditions)
  (:predicates (onTable ?x) (onBurner ?x) (inside ?x ?y) (open ?x) (holding ?x ?y) (clear ?x))

  (:action pickup
    :parameters (?ob ?roboArm)
    :precondition (and (clear ?roboArm) (onTable ?ob))
    :effect (and (holding ?roboArm ?ob) (not (clear ?roboArm)) (not (onTable ?ob)))
  )

  (:action openDrawer
    :parameters (?drawer)
    :precondition (and (clear ?roboArm) (not(open ?drawer)) )
    :effect (open ?drawer)
  )

  (:action putOnTable
    :parameters (?obj ?roboArm)
    :precondition (holding ?roboArm ?ob)
    :effect (and (clear ?roboArm) (onTable ?ob) (not (holding ?roboArm ?ob)))
  )

  (:action putInDrawer
    :parameters (?obj ?roboArm ?drawer)
    :precondition (and (holding ?roboArm ?ob) (open ?drawer))
    :effect (and (clear ?roboArm) (inside ?drawer ?obj) (not (holding ?roboArm ?ob)))
  )
)