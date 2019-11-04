```mermaid
graph LR;
  broker>MQTT Broker]-->router(Command Router);
  router-->pick(Pick Client);
  pick-->move(Move Base Server);
  router-->place(Place Client);
  router-->dock_cl(Dock Undock Client);
  place-->move;
  drive(Drive Client)-->move;
  dock_se(Dock Undock Server)-->dock_cl;
  pick-->dock_po(Dock Pose Server);
  place-->park(Park Pose Server);
  %% pick-->dynam(dynamic reconf server);
  %% dynam-->dock_se;
  subgraph Clients
  pick
  place
  drive
  dock_cl
  end
  subgraph ROS Service Servers
  dock_po
  park
  end
  subgraph ROS Action Servers
  dock_se
  move
  end

```
