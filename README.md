# RoboShopperPrototype
41X Project

#Pre-Defined Traversing
The prototype would have a local look-up table of all the possible routes to its desired destination to perform a pre-defined traversing design. This local look-up table would store directions from the cart dispatch area to the specific checkpoint. The cart will then maneuver to the desired product and wait for input from the user to continue to the next destination (achieving an automation level of 3). After completing all customer requests, the customer is ready to check out the cart maneuvers to the checkout.  

This design has the advantage of being adaptable to specific product location changes since the grocery store can update the product location change or if the product is no longer available. The design has the disadvantage of being reliant on the calibration of the motor-distance algorithm to compute the exact desired location. If the motor/wheels over or underperforms, the cart may be stuck in an edge case that would require user input to correct.
