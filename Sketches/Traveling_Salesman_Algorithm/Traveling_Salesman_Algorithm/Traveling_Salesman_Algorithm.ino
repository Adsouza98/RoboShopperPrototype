float ary[5][5] = {
                    {0.0, 3.4, 3.9, 4.9, 8.2},
                    {3.0, 0.0, 3.95, 6.8, 4.8},
                    {3.8, 4.1, 0.0, 3.8, 4.55},
                    {4.9, 8.0, 4.6, 0.0, 3.2},
                    {7.8, 4.8, 4.55, 2.0, 0.0}
                  };
float completed[10];
float cost = 0;
int costMatrix_Size = 5;

int routingTable[6];
int routingTableIndex = 0;

void setup() {
  Serial.begin(9600);                            //Set Baud rate to 9600
}

void loop() {

  Serial.println("\nThe Path is: ");
  minCost((float*)ary, 0);
  Serial.println("\nMinimum Cost is: ");
  Serial.println(cost);

  Serial.println("The Optimized Route is: ");
  for (int i=0; i<routingTableIndex; i++) {
    Serial.print(routingTable[i]);
    Serial.print("--->");
  }

  float newary[5][5];
  int newSize = 5;
  Serial.println("");
  printArray((float*)ary);

  Serial.println("Removing C1 from array");
  costTableRefactor((float*)newary, 1, -1, -1, -1);
  Serial.println("");
  printArray((float*)newary);

  cost = 0;
  memset(completed, 0, sizeof(completed));
  memset(routingTableIndex, 0, sizeof(routingTableIndex));
  routingTableIndex = 0;

  Serial.println("\nThe Path is: ");
  minCost((float*)newary, 0);
  Serial.println("\nMinimum Cost is: ");
  Serial.println(cost);

  Serial.println("The Optimized Route2 is: ");
  for (int i=0; i<routingTableIndex; i++) {
    Serial.print(routingTable[i]);
    Serial.print("--->");
  }

  while(1){}
}

void minCost(float* a, int checkpoint)
{
  int i;
  int nextCheckpoint;
  completed[checkpoint] = 1;

  Serial.print("Adding Checkpoint: ");
  Serial.println(checkpoint);
  routingTable[routingTableIndex] = checkpoint;
  routingTableIndex++;

  nextCheckpoint = least(a, checkpoint);

  if (nextCheckpoint == 999) {
    nextCheckpoint = 0;
    cost += *((a+checkpoint*costMatrix_Size)+nextCheckpoint);
    Serial.print("Adding Checkpoint: ");
    Serial.println(nextCheckpoint);
    routingTable[routingTableIndex] = nextCheckpoint;
    routingTableIndex++;
    Serial.println("Leaving minCost Function");
    return;
  }
  minCost(a, nextCheckpoint);
}

int least(float* a, int checkpoint)
{
  int nextCheckpoint = 999;
  int i;
  float min = 999, kmin;

  for (i=0;i<costMatrix_Size;i++) {
    if ( ( *((a+checkpoint*costMatrix_Size)+i) != 0) && (completed[i] == 0) ) {
      if (*((a+checkpoint*costMatrix_Size)+i) + *((a+i*costMatrix_Size)+checkpoint) < min) {
        min = *((a+i*costMatrix_Size)+0) + *((a+checkpoint*costMatrix_Size)+i);
        kmin = *((a+checkpoint*costMatrix_Size)+i);
        nextCheckpoint = i;
      }
    }
  }
  if (min != 999) {
    cost += kmin;
  }

  return nextCheckpoint;
}

void costTableRefactor(float* tmp, int a, int b, int c, int d)
{
  for (int i=0; i<costMatrix_Size; i++) {
    for (int j=0; j<costMatrix_Size; j++) {
      if (i == a || i == b || i == c || i == d || j == a || j == b || j == c || j == d) {
        *((tmp+i*costMatrix_Size)+j) = 0.0;
      } else {
        *((tmp+i*costMatrix_Size)+j) = ary[i][j];
      }

    }
  }
}

void printArray(float* a)
{
  for (int i=0; i<costMatrix_Size; i++) {
    for (int j=0; j<costMatrix_Size; j++) {
      Serial.print(*((a+i*costMatrix_Size)+j));
      Serial.print(" ");
    }
    Serial.println("");
  }
}