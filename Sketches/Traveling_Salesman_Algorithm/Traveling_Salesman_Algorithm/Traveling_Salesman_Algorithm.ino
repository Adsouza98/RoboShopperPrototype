float ary[5][5] = {
                    {0.0, 3.4, 3.9, 4.9, 8.2},
                    {3.0, 0.0, 3.95, 6.8, 4.8},
                    {3.8, 4.1, 0.0, 3.8, 4.55},
                    {4.9, 8.0, 4.6, 0.0, 3.2},
                    {7.8, 4.8, 4.55, 2.0, 0.0}
                  };
float ary2[5][5] = {
                    {0.0, 0.0, 3.9, 4.9, 8.2},
                    {0.0, 0.0, 0.0, 0.0, 0.0},
                    {3.8, 0.0, 0.0, 3.8, 4.55},
                    {4.9, 0.0, 4.6, 0.0, 3.2},
                    {7.8, 0.0, 4.55, 2.0, 0.0}
                  };
float completed[10];
float cost = 0;
int n = 5;

void setup() {
  Serial.begin(9600);                            //Set Baud rate to 9600
}

void loop() {

  Serial.println("\nThe Path is: ");
  minCost((float*)ary, n, 0);
  Serial.println("\nMinimum Cost is: ");
  Serial.println(cost);

  float newary[5][5];
  int newSize = 5;
  Serial.println("");
  printArray((float*)ary, 5);

  Serial.println("Removing C1 from array");
  //costTableRefactor((float*)newary, n, newSize, 1, -1, -1, -1);
  costTableRefactor2((float*)newary, 1, -1, -1, -1);
  Serial.println("");
  printArray((float*)newary, n);

  cost = 0;
  memset(completed, 0, sizeof(completed));

  Serial.println("\nThe Path is: ");
  minCost((float*)newary, newSize, 0);
  //minCost((float*)ary2, 5, 0);
  Serial.println("\nMinimum Cost is: ");
  Serial.println(cost);

  while(1){}
}

void minCost(float* a, int size, int checkpoint)
{
  int i;
  int nextCheckpoint;
  completed[checkpoint] = 1;

  Serial.print(checkpoint);
  Serial.print("--->");
  nextCheckpoint = least(a, size, checkpoint);

  if (nextCheckpoint == 999) {
    nextCheckpoint = 0;
    Serial.print(nextCheckpoint);
    cost += *((a+checkpoint*size)+nextCheckpoint);
    return;
  }
  minCost(a, size, nextCheckpoint);
}

int least(float* a, int size, int checkpoint)
{
  int nextCheckpoint = 999;
  int i;
  float min = 999, kmin;

  for (i=0;i<size;i++) {
    if ( ( *((a+checkpoint*size)+i) != 0) && (completed[i] == 0) ) {
      if (*((a+checkpoint*size)+i) + *((a+i*size)+checkpoint) < min) {
        min = *((a+i*size)+0) + *((a+checkpoint*size)+i);
        kmin = *((a+checkpoint*size)+i);
        nextCheckpoint = i;
      }
    }
  }
  if (min != 999) {
    cost += kmin;
  }

  return nextCheckpoint;
}

void costTableRefactor(float* tmp, int size, int tmpSize, int a, int b, int c, int d)
{
  int row = 0;
  int col = 0;

  Serial.println("REFACTOR START");
  for (int i=0; i<size; i++) {
    if (i == a || i == b || i == c || i == d) {
      row--;
    }
    col = 0;
    for (int j=0; j<size; j++) {
      if (j == a || j == b || j == c || j == d) {
        col--;
      }
      if (i != a && i != b && i != c && i != d && j != a && j != b && j != c && j != d) {
        *((tmp+row*tmpSize)+col) = ary[i][j];
      }
      col++;
    }
    Serial.println("");
    row++;
  }
  Serial.println("REFACTOR END");
}

void costTableRefactor2(float* tmp, int a, int b, int c, int d)
{
  Serial.println("REFACTOR START");
  for (int i=0; i<n; i++) {
    for (int j=0; j<n; j++) {
      if (i == a || i == b || i == c || i == d || j == a || j == b || j == c || j == d) {
        *((tmp+i*n)+j) = 0.0;
      } else {
        *((tmp+i*n)+j) = ary[i][j];
      }

    }
  }
  Serial.println("REFACTOR END");
}

void printArray(float* a, int size)
{
  for (int i=0; i<size; i++) {
    for (int j=0; j<size; j++) {
      Serial.print(*((a+i*size)+j));
      Serial.print(" ");
    }
    Serial.println("");
  }
}