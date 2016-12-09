var boids = [];
var classes = ["Photography 1", "Photography 2", "Photography 3", "Video Production 1", "Video Production 2", "blah","blah","blah", "blah","blah","blah","blah","blah","blah","blah","blah","blah"];
var pages = [];
function setup() {
  var width = document.getElementById('head-img').offsetWidth;
  var height = document.getElementById('head-img').offsetHeight ;
  var myCanvas = createCanvas(width,height);
  myCanvas.parent('head-img');
  // Add an initial set of boids into the system
  for (var i = 0; i < 16; i++) {
    boids[i] = new Boid(random(width), random(height));
  }
}

function draw() {
  clear();
  // Run all the boids
  for (var i = 0; i < boids.length; i++) {
    boids[i].run(boids);
      rectMode(CENTER);
      textAlign(CENTER);
      textSize(20);
      fill(255,255,255);
      text(classes[i], boids[i].position.x, boids[i].position.y, 75, 75);
  }
  dragBoid();
  redirect();
}

function dragBoid(){
  for(var i = 0; i < boids.length; i++){
    hit = collidePointCircle(mouseX,mouseY,boids[i].position.x,boids[i].position.y,150);
    if(hit && mouseIsPressed){
      boids[i].position.x = mouseX;
      boids[i].position.y = mouseY;
      //window.location = "http://www.google.com/"
    }
  }
}

function redirect(){}

function windowResized() {
  resizeCanvas(document.getElementById('head-img').offsetWidth, document.getElementById('head-img').offsetHeight);
}

// Boid class
// Methods for Separation, Cohesion, Alignment added
function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.velocity = p5.Vector.random2D();
  this.position = createVector(x, y);
  this.r = 6.0;
  this.maxspeed = 0.5;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
}

Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.borders();
  this.render();
}

// Forces go into acceleration
Boid.prototype.applyForce = function(force) {
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  var sep = this.separate(boids); // Separation
  var ali = this.align(boids);    // Alignment
  var coh = this.cohesion(boids); // Cohesion
  // Arbitrarily weight these forces
  sep.mult(5);
  ali.mult(0);
  coh.mult(0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
  // Reset acceleration to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  var desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired, this.velocity);
  steer.limit(this.maxforce); // Limit to maximum steering force
  return steer;
}

// Draw boid as a circle
Boid.prototype.render = function() {
  fill(114, 203, 207, 75);
  noStroke();
  ellipse(this.position.x, this.position.y, 150,150);
}

// Wraparound
Boid.prototype.borders = function() {

  if ((this.position.x + (150/2)) > width || (this.position.x - (150/2)) < 0) {
    this.velocity.x *= (-1);
  }

  if ((this.position.y + (150/2)) > height || (this.position.y - (150/2)) < 0) {
    this.velocity.y *= (-1);
  }
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  var desiredseparation = 150.0;
  var steer = createVector(0, 0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d); // Weight by distance
      steer.add(diff);
      count++; // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  var neighbordist = 50;
  var sum = createVector(0, 0);
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  var neighbordist = 50;
  var sum = createVector(0, 0); // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum); // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}
