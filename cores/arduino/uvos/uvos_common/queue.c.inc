// This is a C module file that implements a queue data structure using a linked list
// A queue is a linear collection of items that follows the FIFO (first-in first-out) principle
// The queue has two ends: front and rear. Items are added at the rear and removed from the front

#include <stdio.h>
#include <stdlib.h>

// Define a node structure for the linked list
struct node {
  int data; // The data stored in the node
  struct node *next; // The pointer to the next node
};

// Define a queue structure that contains a pointer to the front and rear nodes and the size of the queue
struct queue {
  struct node *front; // The pointer to the front node
  struct node *rear; // The pointer to the rear node
  int size; // The number of items in the queue
};

// Create a new queue and return a pointer to it
struct queue *create_queue() {
  // Allocate memory for the queue structure
  struct queue *q = (struct queue *)malloc(sizeof(struct queue));
  // Check if the memory allocation was successful
  if (q == NULL) {
    printf("Memory allocation failed.\n");
    exit(1);
  }
  // Initialize the front and rear pointers to NULL and the size to 0
  q->front = NULL;
  q->rear = NULL;
  q->size = 0;
  // Return the pointer to the queue
  return q;
}

// Check if the queue is empty and return a boolean value
int is_empty(struct queue *q) {
  // The queue is empty if the size is 0
  return (q->size == 0);
}

// Add an item at the rear of the queue
void enqueue(struct queue *q, int item) {
  // Allocate memory for a new node
  struct node *new_node = (struct node *)malloc(sizeof(struct node));
  // Check if the memory allocation was successful
  if (new_node == NULL) {
    printf("Memory allocation failed.\n");
    exit(1);
  }
  // Set the data and next fields of the new node
  new_node->data = item;
  new_node->next = NULL;
  // If the queue is empty, set both front and rear pointers to point to the new node
  if (is_empty(q)) {
    q->front = new_node;
    q->rear = new_node;
  }
  // Otherwise, set the next field of the current rear node to point to the new node and update the rear pointer
  else {
    q->rear->next = new_node;
    q->rear = new_node;
  }
  // Increment the size of the queue by one
  q->size++;
}

// Remove an item from the front of the queue and return its value
int dequeue(struct queue *q) {
  // Check if the queue is empty
  if (is_empty(q)) {
    printf("Queue underflow.\n");
    exit(1);
  }
  // Get a pointer to the front node and store its data value
  struct node *temp = q->front;
  int item = temp->data;
  // Set the front pointer to point to the next node in the list
  q->front = q->front->next;
  // If the queue becomes empty, set both front and rear pointers to NULL
  if (q->front == NULL) {
    q->rear = NULL;
  }
  // Free the memory allocated for the removed node
  free(temp);
  // Decrement the size of the queue by one
  q->size--;
  // Return the data value of the removed node
  return item;
}

// Get the value of the front item without removing it from the queue
int peek(struct queue *q) {
   // Check if the queue is empty
   if (is_empty(q)) {
     printf("Queue underflow.\n");
     exit(1);
   }
   // Return the data value of the front node
   return q->front->data;
}

// Print all items in the queue from front to rear
void print_queue(struct queue *q) {
   // Check if the queue is empty
   if (is_empty(q)) {
     printf("Queue is empty.\n");
     return;
   }
   // Get a pointer to the front node and traverse the list until the rear node
   struct node *current = q->front;
   while (current != NULL) {
     // Print the data value of the current node
     printf("%d ", current->data);
     // Move to the next node
     current = current->next;
   }
   // Print a new line
   printf("\n");
}

// Delete the queue and free the memory allocated for it
void delete_queue(struct queue *q) {
  // Check if the queue is empty
  if (is_empty(q)) {
    // Free the memory allocated for the queue structure
    free(q);
    return;
  }
  // Get a pointer to the front node and traverse the list until the rear node
  struct node *current = q->front;
  while (current != NULL) {
    // Get a pointer to the next node
    struct node *next = current->next;
    // Free the memory allocated for the current node
    free(current);
    // Move to the next node
    current = next;
  }
  // Free the memory allocated for the queue structure
  free(q);
}