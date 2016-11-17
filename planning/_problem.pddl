(define (problem emergency-template)
(:domain emergency)
(:objects
  turtlebot1 - turtlebot
  turtlebot2 - turtlebot
  drone1 - drone
  drone2 - drone
  w1 w10 w11 w12 w13 w2 w3 w4 w5 w6 w7 w8 w9 mesh1 mesh2 mesh3 mesh4 mesh5 mesh6 mesh7 mesh8 mesh9 mesh10 mesh11 mesh12 mesh13 - waypoint
  w1_air w10_air w11_air w12_air w13_air w2_air w3_air w4_air w5_air w6_air w7_air w8_air w9_air mesh1_air mesh2_air mesh3_air mesh4_air mesh5_air mesh6_air mesh7_air mesh8_air mesh9_air mesh10_air mesh11_air mesh12_air mesh13_air - airwaypoint
  box1 box2 box3 box4 box4 - box
  person1 person2 person3 person4 - person)

(:init
  (= (move-duration w1 w7) 1.24949989996)
  (= (move-duration w7 w1) 1.24949989996)
  (= (move-duration w1_air w7_air) 0.832999933307)
  (= (move-duration w7_air w1_air) 0.832999933307)
  (= (move-duration w1 w2) 0.289352725925)
  (= (move-duration w2 w1) 0.289352725925)
  (= (move-duration w1_air w2_air) 0.192901817283)
  (= (move-duration w2_air w1_air) 0.192901817283)
  (= (move-duration w1 w4) 0.967845545529)
  (= (move-duration w4 w1) 0.967845545529)
  (= (move-duration w1_air w4_air) 0.645230363686)
  (= (move-duration w4_air w1_air) 0.645230363686)
  (= (move-duration w10 w8) 1.05798156884)
  (= (move-duration w8 w10) 1.05798156884)
  (= (move-duration w10_air w8_air) 0.705321045892)
  (= (move-duration w8_air w10_air) 0.705321045892)
  (= (move-duration w10 w11) 1.51000827812)
  (= (move-duration w11 w10) 1.51000827812)
  (= (move-duration w10_air w11_air) 1.00667218542)
  (= (move-duration w11_air w10_air) 1.00667218542)
  (= (move-duration w11 w9) 1.95156988089)
  (= (move-duration w9 w11) 1.95156988089)
  (= (move-duration w11_air w9_air) 1.30104658726)
  (= (move-duration w9_air w11_air) 1.30104658726)
  (= (move-duration w11 w8) 2.35483545073)
  (= (move-duration w8 w11) 2.35483545073)
  (= (move-duration w11_air w8_air) 1.56989030048)
  (= (move-duration w8_air w11_air) 1.56989030048)
  (= (move-duration w11 w6) 3.37239529118)
  (= (move-duration w6 w11) 3.37239529118)
  (= (move-duration w11_air w6_air) 2.24826352746)
  (= (move-duration w6_air w11_air) 2.24826352746)
  (= (move-duration w12 w2) 3.74080539456)
  (= (move-duration w2 w12) 3.74080539456)
  (= (move-duration w12_air w2_air) 2.49387026304)
  (= (move-duration w2_air w12_air) 2.49387026304)
  (= (move-duration w12 w5) 3.6166973885)
  (= (move-duration w5 w12) 3.6166973885)
  (= (move-duration w12_air w5_air) 2.41113159233)
  (= (move-duration w5_air w12_air) 2.41113159233)
  (= (move-duration w13 w8) 2.09134526083)
  (= (move-duration w8 w13) 2.09134526083)
  (= (move-duration w13_air w8_air) 1.39423017389)
  (= (move-duration w8_air w13_air) 1.39423017389)
  (= (move-duration w13 w5) 2.28758606395)
  (= (move-duration w5 w13) 2.28758606395)
  (= (move-duration w13_air w5_air) 1.52505737597)
  (= (move-duration w5_air w13_air) 1.52505737597)
  (= (move-duration w2 w5) 1.06424856119)
  (= (move-duration w5 w2) 1.06424856119)
  (= (move-duration w2_air w5_air) 0.709499040795)
  (= (move-duration w5_air w2_air) 0.709499040795)
  (= (move-duration w2 w3) 0.742041103983)
  (= (move-duration w3 w2) 0.742041103983)
  (= (move-duration w2_air w3_air) 0.494694069322)
  (= (move-duration w3_air w2_air) 0.494694069322)
  (= (move-duration w3 w7) 1.07074740252)
  (= (move-duration w7 w3) 1.07074740252)
  (= (move-duration w3_air w7_air) 0.713831601679)
  (= (move-duration w7_air w3_air) 0.713831601679)
  (= (move-duration w3 w6) 1.3680003655)
  (= (move-duration w6 w3) 1.3680003655)
  (= (move-duration w3_air w6_air) 0.912000243665)
  (= (move-duration w6_air w3_air) 0.912000243665)
  (= (move-duration w4 w9) 1.0747674167)
  (= (move-duration w9 w4) 1.0747674167)
  (= (move-duration w4_air w9_air) 0.716511611133)
  (= (move-duration w9_air w4_air) 0.716511611133)
  (= (move-duration w4 w6) 0.37947331922)
  (= (move-duration w6 w4) 0.37947331922)
  (= (move-duration w4_air w6_air) 0.252982212813)
  (= (move-duration w6_air w4_air) 0.252982212813)
  (= (move-duration w6 w8) 1.14004385881)
  (= (move-duration w8 w6) 1.14004385881)
  (= (move-duration w6_air w8_air) 0.760029239204)
  (= (move-duration w8_air w6_air) 0.760029239204)
  (= (move-duration w6 w7) 0.860363295358)
  (= (move-duration w7 w6) 0.860363295358)
  (= (move-duration w6_air w7_air) 0.573575530239)
  (= (move-duration w7_air w6_air) 0.573575530239)

  (over w1_air w1)
  (over w10_air w10)
  (over w11_air w11)
  (over w12_air w12)
  (over w13_air w13)
  (over w2_air w2)
  (over w3_air w3)
  (over w4_air w4)
  (over w5_air w5)
  (over w6_air w6)
  (over w7_air w7)
  (over w8_air w8)
  (over w9_air w9)
  (over mesh1_air mesh1)
  (over mesh2_air mesh2)
  (over mesh3_air mesh3)
  (over mesh4_air mesh4)
  (over mesh5_air mesh5)
  (over mesh6_air mesh6)
  (over mesh7_air mesh7)
  (over mesh8_air mesh8)
  (over mesh9_air mesh9)
  (over mesh10_air mesh10)
  (over mesh11_air mesh11)
  (over mesh12_air mesh12)
  (over mesh13_air mesh13)

  (free box1)
  (at box1 w5)
  (free box2)
  (at box2 w6)
  (free box3)
  (at box3 w7)
  (free box4)
  (at box4 w8)
  (free box4)
  (at box4 w9)

  (empty turtlebot1)
  (at turtlebot1 w1)

  (empty turtlebot2)
  (at turtlebot2 w2)

  (empty drone1)
  (at drone1 w3_air)

  (empty drone2)
  (at drone2 w4_air)


  (empty mesh6_air)
  (empty mesh1_air)
  (empty mesh4_air)
  (empty w13)
  (empty w12)
  (empty w11)
  (empty w10)
  (empty w7)
  (empty w6)
  (empty w5)
  (empty w4)
  (empty w3)
  (empty mesh8_air)
  (empty w9)
  (empty w8)
  (empty mesh9_air)
  (empty mesh12_air)
  (empty w11_air)
  (empty w8_air)
  (empty w5_air)
  (empty mesh10_air)
  (empty w13_air)
  (empty mesh3_air)
  (empty mesh13_air)
  (empty w7_air)
  (empty w6_air)
  (empty w12_air)
  (empty mesh2_air)
  (empty mesh11_air)
  (empty w2_air)
  (empty w9_air)
  (empty mesh5_air)
  (empty w1_air)
  (empty mesh12)
  (empty mesh13)
  (empty mesh10)
  (empty mesh11)
  (empty mesh7_air)
  (empty mesh1)
  (empty mesh2)
  (empty mesh3)
  (empty mesh4)
  (empty mesh5)
  (empty mesh6)
  (empty mesh7)
  (empty mesh8)
  (empty mesh9)
  (empty w10_air)

  (occupied w4_air)
  (occupied w2)
  (occupied w1)
  (occupied w3_air)

  (at person1 w10)
  (at person2 w11)
  (at person3 w12)
  (at person4 w13)
)

(:goal (and
            (handled person1)
            (handled person2)
            (handled person3)
            (handled person4)
)))