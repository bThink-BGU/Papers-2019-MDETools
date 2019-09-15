importPackage(Packages.java.lang);
importPackage(Packages.il.ac.bgu.cs.bp.leaderfollower.events);
importPackage(Packages.il.ac.bgu.cs.bp.leaderfollower.schema);
importPackage(Packages.il.ac.bgu.cs.bp.bpjs.context);

var player;
var opponent;

var MIN_DEGREE = 5;
var MAX_SPIN = 50;
var MAX_PWR = 100;
var TOO_FAR = 5;
var TOO_CLOSE = 3.7;

var FBWARD_EVENT_REGEX = /^(Possesion|Timeout|scored|Done)/
var refereeEvents = bp.EventSet("RefereeEvents", function (e) {
  return e.name.match(FBWARD_EVENT_REGEX) !== null;
});
var scoreEvents = bp.EventSet("RefereeScoreEvents", function (e) {
  return e.name.equals("scored");
});
var moveEvents = bp.EventSet("MoveEvents", function (e) {
  return e instanceof ParameterizedMove && (e.powerForward != null || e.powerLeft != null);
});
var spinEvents = bp.EventSet("MoveEvents", function (e) {
  return e instanceof ParameterizedMove && e.spin != null;
});
var anyParameterizedMove = bp.EventSet("AnyParameterizedMove", function (e) {
  return e instanceof ParameterizedMove;
});

//#region helper functions
function needGradient(distance) {
  return (distance > TOO_CLOSE && distance < TOO_FAR) || (distance < TOO_CLOSE && distance > (2 * TOO_CLOSE - TOO_FAR));
}

function gradient(distance) {
  return Math.round(((distance - TOO_CLOSE) / (TOO_FAR - TOO_CLOSE)) * 100);
}

function suction() {
  return bp.Event("Suck");
}

function expel() {
  return bp.Event("Expel");
}

function spin(power) {
  return ParameterizedMove(null, null, power);
}

function moveForward(power) {
  return ParameterizedMove(power, null, null);
}

function distanceFromPlayer(state) {
  return state.distance;
}

function degreeFromPlayer(state) {
  return state.degree;
}
//#endregion

bp.registerBThread("HandleRefereeEvents", function () {
  while (true) {
    var e = bp.sync({ waitFor: refereeEvents });
    if (e.name.equals("Possesion")) {
      bp.sync({ request: CTX.UpdateEvent("UpdatePosession", { "posession": e.data[1] }) });
    } else if (e.name.equals("Timeout")) {
      bp.sync({ request: CTX.UpdateEvent("UpdateTimeout", { "timeout": e.data.counter }) });
    } else if (e.name.equals("scored")) {
      if (e.data[1].equals(player.name))
        bp.sync({ request: CTX.TransactionEvent(
            CTX.UpdateEvent("UpdatePosession", { "posession": "" }), 
            CTX.UpdateEvent("UpdateMyScore", { "score": Double(e.data[2]).intValue() }) 
        )});
      else
        bp.sync({ request: CTX.TransactionEventr(
            CTX.UpdateEvent("UpdatePosession", { "posession": "" }), 
            CTX.UpdateEvent("UpdateOpponentScore", { "score": Double(e.data[2]).intValue() })
        )});
    } else if (e.name.equals("Done")) {
      bp.sync({ request: CTX.UpdateEvent("MarkGameAsOver") });
    } else
      bp.log.ERROR("unknown message from referee");
  }
});

bp.registerBThread("InitData", function () {
  bp.sync({ waitFor: bp.Event("Start Control") });
  bp.sync({ request: CTX.InsertEvent(Referee(player.name), Target()) });
});

CTX.subscribe("MoveTowardsTarget", "MoveTowardsTarget", function (target) {
  // var ctxEndedEvent = CTX.AnyContextEndedEvent("BallIsFree");
  var distance = target.distanceFromPlayer;
  if (needGradient(distance))
    bp.sync({ request: moveForward(gradient(distance)), waitFor:  spinEvents});
  else
    bp.sync({ request: moveForward(MAX_PWR), waitFor: spinEvents });
});

CTX.subscribe("SpinToTarget", "MoveTowardsTarget", function (target) {
  var degree = target.degreeFromPlayer;
  if (degree < -MIN_DEGREE)
    bp.sync({ request: spin(-MAX_SPIN), block: moveEvents });
  else if (degree > MIN_DEGREE)
    bp.sync({ request: spin(MAX_SPIN), block: moveEvents });
  /* else
    bp.sync({ request: spin(0), block: moveEvents }); */
});

CTX.subscribe("BallSuction", "BallIsFreeNearPlayer", function (target) {
//CTX.subscribe("BallSuction", "MoveTowardsBall", function (referee) {
  bp.sync({ request: suction() });
  bp.sync({ block: suction(), waitFor: [expel(), scoreEvents] });
  var i;
  for (i=0; i<2; i++) 
    bp.sync({ block: suction(), waitFor: StateUpdate.ANY });
});

CTX.subscribe("UpdateBallTarget", "BallIsFree", function (referee) {
  var ctxFreeEndedEvent = CTX.AnyContextEndedEvent("BallIsFree");
  while(true) {
    var ball = bp.sync({waitFor: StateUpdate.ANY, interrupt: ctxFreeEndedEvent}).ball;
    bp.sync({
      request: 
      CTX.TransactionEvent(
        CTX.UpdateEvent("PurgeOldTargets"),
        CTX.InsertEvent(new Target("ball", distanceFromPlayer(ball), degreeFromPlayer(ball)))
      )
    });
  }
});

// ########################################################################################333

/* bp.registerBThread("block2suck", function(){
  while(true) {
    bp.sync({ waitFor: expel() });
    bp.sync({ block: suction(), waitFor: expel() });
  }
}); */

CTX.subscribe("UpdateGoalTarget", "IPossesTheBall", function (referee) {
  var contextEndedEvent = CTX.AnyContextEndedEvent("IPossesTheBall", referee);
  while(true) {
    var goal = bp.sync({waitFor: StateUpdate.ANY, interrupt: contextEndedEvent}).goal;
    bp.sync({
      request: 
      CTX.TransactionEvent(
        CTX.UpdateEvent("PurgeOldTargets"),
        CTX.InsertEvent(new Target("goal", distanceFromPlayer(goal), degreeFromPlayer(goal)))
      )
    });
  }
});

CTX.subscribe("ReleaseTheBallWhenReady", "FacingGoal", function (target) {
  bp.sync({ request: expel() });
  bp.sync({ request: CTX.UpdateEvent("UpdatePosession", { "posession": "" }) });
});

CTX.subscribe("ChangeSpinPowerWhenFree", "BallIsFree", function (referee) {
  MAX_SPIN = 50;
});

CTX.subscribe("ChangeSpinPowerWhenIPosses", "IPossesTheBall", function (referee) {
  MAX_SPIN = 100;
});