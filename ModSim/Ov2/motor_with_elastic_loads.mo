within ;
model motor_with_elastic_loads
  Modelica.Mechanics.Rotational.Sources.Torque torque
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia motor_inertia(J=1)
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=0.5, d=
        0.01) annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1)
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1)
    annotation (Placement(transformation(extent={{120,0},{140,20}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper1(c=0.5, d=
        0.01) annotation (Placement(transformation(extent={{80,0},{100,20}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{160,0},{180,20}})));
  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-120,0},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{200,0},{220,20}})));
equation
  connect(motor_inertia.flange_a, torque.flange) annotation (Line(
      points={{-40,10},{-60,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(springDamper.flange_a, motor_inertia.flange_b) annotation (Line(
      points={{0,10},{-20,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia1.flange_a, springDamper.flange_b) annotation (Line(
      points={{40,10},{20,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(springDamper1.flange_a, inertia1.flange_b) annotation (Line(
      points={{80,10},{60,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(springDamper1.flange_b, inertia2.flange_a) annotation (Line(
      points={{100,10},{120,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(speedSensor.flange, inertia2.flange_b) annotation (Line(
      points={{160,10},{140,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(torque.tau, u) annotation (Line(
      points={{-82,10},{-110,10}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(speedSensor.w, y) annotation (Line(
      points={{181,10},{210,10}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (
    uses(Modelica(version="3.2.1")),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{
            220,100}}), graphics),
    Icon(coordinateSystem(extent={{-120,-100},{220,100}})));
end motor_with_elastic_loads;
