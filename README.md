# ASR-DeliverPackage
Repo for ASR final project, our project consists in making the kobuki deliver packages to people.

## Behavior Tree
![schemaBT](https://github.com/user-attachments/assets/4aff3e78-ae81-4dd2-96fe-426bb0ed0d31)

### BT's Description
This is the kobuki's delivery workflow:
1) **Check for Summons**: The kobuki first waits until it's summoned (_isSummoned_ condition)
2) **Setup Delivery**: The kobuki receives all the infomation neccessary (_destination_ and _receiver_) and initializes a delivery operation
3) **Execute Delivery**: Carries out the actual delivery process through two main phases:
  - **Travel Phase:** The robot attempts to travel to the specified destination. If that fails, returns to home base
  - **Search and Deliver Phase:** After reaching the destination, the robot:
    - Tries up to 5 times (_num\_attempts_) to:
      - Search a person
      - Verify it's the actual receiver and then deliver the package (all failures are expected to occur here)
      - Return to home base
    - If all attempts fail (receiver not found), it returns to home base
