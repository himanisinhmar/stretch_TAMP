Robot manipulation project - manipulating a new environment based on affordances

## Install planner
`wget https://bitbucket.org/ipc2018-classical/team1/raw/ipc-2018-seq-sat/Singularity` \
`sudo singularity build planner.img Singularity` \
If there are any installation issues, check out the [source](https://ipc2018-classical.bitbucket.io/index.html#planners).

## Download the repo
`git clone https://github.com/sguysc/CS6751_project.git`

## Run
<ol>
  <li>Edit the file "encode_pddl_inputs.py" for different user defined specifications.</li>
  <li>Edit "environment.json" to change affordances, objects or properties.</li>
  <li>Run the python file</li>
  <li>Run ./runit.bash (after fixing for the correct paths within it)</li>
  <li>The result action sequence is in the file "sas_plan".</li>
</ol>

## To run generate_waypoints.py
<ol> 
  <li> Install [PythonRobotics](https://pythonrobotics.readthedocs.io/en/latest/index.html#) and make sure it's in the correct path
  <li> Additional required libraries: scipy, shapely
</ol>
    
