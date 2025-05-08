# ASR-DeliverPackage
Repo for the ASR final project. Our project consists of making the Kobuki deliver packages to people.

## Behavior Tree
_**NOTE:** This is a general idea of how the final behavior tree will look._

![schemaBT](https://github.com/user-attachments/assets/4aff3e78-ae81-4dd2-96fe-426bb0ed0d31)

### BT's Description
This is the Kobuki's delivery workflow:
1) **Setup Delivery**: The Kobuki receives all the necessary information (_destination_ and _receiver_) and initializes a delivery operation.
   - We check if the destination name is registered in the list of waypoints.
   - The expected information about the receiver is their first name and last name.

2) **Execute Delivery**: Carries out the actual delivery process through two main phases:
   - **Travel Phase:** The robot attempts to travel to the specified destination. If that fails, it returns to the home base.
     - The waypoint should be at the entrance of the destination, so when it arrives, it changes its state to DeliverConfirmation.
   - **Search and Deliver Phase:** After reaching the destination, the robot:
     - Tries up to 5 times (_num_attempts_) to:
       - Verify the actual receiver and deliver the package
       - Return to the home base
     - If all attempts fail (receiver not found), it returns to the home base
     - Once it reaches the destination, it starts loudly asking for the expected receiver’s name.
     - When someone responds, it asks for their last name (which acts as a "password") to verify if they are the correct person. If not, or if there’s a misunderstanding or no answer, it repeats the process several times. If the receiver is definitely not there, it returns to the base.
     - When it finds the receiver, it gives a green light to pick up the package and then returns to the base.
