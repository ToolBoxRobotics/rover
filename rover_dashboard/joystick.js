class Joystick {
  constructor(containerId, size, callback) {
    this.callback = callback;
    let container = document.getElementById(containerId);

    let canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    container.appendChild(canvas);

    let ctx = canvas.getContext("2d");
    let radius = size/2;

    let state = {x:0, y:0, active:false};

    function draw() {
      ctx.clearRect(0,0,size,size);

      // Outer ring
      ctx.strokeStyle = "#555";
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.arc(radius, radius, radius-4, 0, 2*Math.PI);
      ctx.stroke();

      // Joystick knob
      ctx.fillStyle = "#0f0";
      ctx.beginPath();
      ctx.arc(radius + state.x*(radius-20),
              radius - state.y*(radius-20),
              20, 0, 2*Math.PI);
      ctx.fill();
    }

    function updateMovement(evt) {
      if (!state.active) return;
      let rect = canvas.getBoundingClientRect();
      let mx = evt.clientX - rect.left;
      let my = evt.clientY - rect.top;

      let dx = (mx - radius)/(radius-20);
      let dy = (radius - my)/(radius-20);

      dx = Math.max(-1, Math.min(1, dx));
      dy = Math.max(-1, Math.min(1, dy));

      state.x = dx;
      state.y = dy;

      callback(state.x, state.y);
      draw();
    }

    canvas.onmousedown = () => { state.active = true; };
    canvas.onmouseup   = () => { state.active = false; state.x=0; state.y=0; callback(0,0); draw(); };
    canvas.onmousemove = updateMovement;

    draw();
  }
}

