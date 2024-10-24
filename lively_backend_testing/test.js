function InitializationExample(props) {
    const [livelySolver, setLivelySolver] = useState(null);
    const [robot, setRobot] = useState("panda");
    const [robotState, setRobotState] = useState(null);
    const [showCollision, setShowCollision] = useState(false);
  
    useEffect(() => {
      /* 
        Given that we are showing this example in a declarative
        react context, we need to use the useEffect hook to execute
        imperative (sequential) code. That means that if you are
        writing standard javascript, your code will look like the
        contents of the "init" function.
        * Note also that the "init" function is async. This is
        because the lively library is built on web assembly (WASM),
        which needs to be imported asynchronously.
        */
      // Instantiate a new solver
      const newSolver = new lively.Solver(
        urdfs[robot], // The urdf of the robot
        {
          smoothness: {
            // An example objective (smoothness macro)
            name: "MySmoothnessObjective",
            type: "SmoothnessMacro",
            weight: 5,
          },
        }
      );
      // Assign the solver to the value
      setLivelySolver(newSolver);
      // Run solve to get a solved state
      const newState = newSolver.solve({}, {}, 0.0);
      // Update the solver's current state
      setRobotState(newState);
  
      return () => {
        // Provide a function to clear previous values
        setLivelySolver(null);
        setRobotState(null);
      };
    }, [robot]); // Rerun this code if the robot changes
  
    return (
      <div>
        <RobotViewer
          state={robotState}
          links={livelySolver ? livelySolver.links : []}
          showCollision={showCollision}
          levaOptions={{
            robot: {
              label: "Robot",
              value: robot,
              options: { UR3e: "ur3e", Panda: "panda"},
              onChange: (r) => setRobot(r),
            },
            showCollision:{
              value:showCollision,
              label:'Show Collisions',
              onChange:(v)=>setShowCollision(v)
            }
          }}
        />
        <Tree label="state" data={robotState} />
      </div>
    );
  }