from __future__ import annotations

from typing import TYPE_CHECKING


from .. import constants
from .core import LpSolver,  PulpSolverError, clock, log



class Knitro(LpSolver):
    """
    The Knitro LP/MIP solver (via its python interface)
    """

    name = "Knitro"
    try:
        # to import the name into the module scope
        global kn
        import knitro as kn  # type: ignore[import-not-found, import-untyped, unused-ignore]
    except:  
        def available(self):
            """True if the solver is available"""
            return False

        def actualSolve(self, lp, callback=None):
            """Solve a well formulated lp problem"""
            raise PulpSolverError("Knitro: Not Available")

    else:

        def __init__(
            self,
            mip=True,
            msg=True,
            timeLimit=None,
            gapRel=None,
            warmStart=False,
            printKnitroOutput=False, # If true print knitro output to console 
            logFile=False, # If true print log in the "knitro.log" file
            **solverParams,
        ):
            """
            :param bool mip: if False, assume LP even if integer variables
            :param bool msg: if False, no log is shown
            :param float timeLimit: maximum time for solver (in seconds)
            :param float gapRel: relative gap tolerance for the solver to stop (in fraction)
            :param bool warmStart: if True, the solver will use the current value of variables as a start
            :param str logPath: path to the log file            :param dict envOptions: environment options.


            
            """
            self.solver_params = solverParams
            self.status= None
            self.model = None
            self.init_Knitro = False  # whether env and model have been initialised

            LpSolver.__init__(
                self,
                mip=mip,
                msg=msg,
                timeLimit=timeLimit,
                gapRel=gapRel,
                printKnitroOutput=printKnitroOutput,
                logFile=logFile,
                warmStart=warmStart,
            )




        def findSolutionValues(self, lp):
            solutionStatus, _, x, _  =  kn.KN_get_solution (lp.solverModel)
            # map knitro status to pulp status: there are many knitro status codes, I group them here
            if solutionStatus >=-199:
                status = constants.LpStatusOptimal
            elif solutionStatus >= -299:
                status = constants.LpStatusInfeasible
            elif solutionStatus >= -399:
                status = constants.LpStatusUnbounded
            elif solutionStatus >= -410:
                status = constants.LpStatusOptimal
            elif solutionStatus >= -499:
                status = constants.LpStatusInfeasible
            else:
                status = constants.LpStatusUndefined
            lp.assignStatus(status)
            if self.msg:
                print("Knitro status=", solutionStatus)
            lp.resolveOK = True
            for var in lp._variables:
                var.isModified = False
            if len(x) >= 1:
                # populate pulp solution values
                for var, value in zip(
                    lp._variables,x
                ):
                    var.varValue = value
                # populate pulp constraints slack
                for constr in lp.constraints.values():
                    # for each constraint get the slack by computing the left hand side - right hand side
                    lhs_value = sum(
                        coef * var.varValue for var, coef in constr.items()
                    )
                    constr.slack = -lhs_value - constr.constant

                # put pi and slack variables against the constraints
                # Populate dual values (pi) and reduced costs (dj) if not a MIP
                if not lp.isMIP():
                    lambda_ = kn.KN_get_con_dual_values(lp.solverModel)
                    dj = kn.KN_get_var_dual_values(lp.solverModel)
                    count = 0
                    for constraint in lp.constraints.values():
                        if count < len(lambda_):
                            constraint.pi = lambda_[count]
                        count += 1
                    count = 0
                    for var in lp._variables:
                        var.dj = dj[count]
                        count += 1
            return status


        def initKnitro(self):
            if self.init_Knitro:
                return
            else:
                self.init_Knitro = True
            try:
                self.model = kn.KN_new()
            except Exception as e:
                raise PulpSolverError("Knitro: Unable to find valid license!") from e


        def callSolver(self, lp, callback=None):
            """Solves the problem with Knitro"""
            # solve the problem
            self.solveTime = -clock()
            kn.KN_solve(lp.solverModel)
            self.solveTime += clock()


        def buildSolverModel(self, lp):
            """
            Takes the pulp lp model and translates it into a knitro model
            """
            log.debug("create the knitro model")
            self.initKnitro()
            lp.solverModel = self.model
            log.debug("set the sense of the problem")
            if lp.sense == constants.LpMaximize:
                kn.KN_set_obj_goal(lp.solverModel, kn.KN_OBJGOAL_MAXIMIZE)
            if self.timeLimit:
                kn.KN_set_double_param(lp.solverModel, kn.KN_PARAM_MAXTIME, self.timeLimit)
                self.model.KN_PARAM_MAXTIME = self.timeLimit
            gapRel = self.optionsDict.get("gapRel")
            logFile = self.optionsDict.get("logFile")
            printKnitroOutput = self.optionsDict.get("printKnitroOutput")
            if gapRel:
                kn.KN_set_double_param(lp.solverModel, kn.KN_PARAM_MIP_OPTGAPREL,gapRel)
            if logFile and printKnitroOutput:
                kn.KN_set_int_param(lp.solverModel, kn.KN_PARAM_OUTMODE,kn.KN_OUTMODE_BOTH)
            elif logFile:
                kn.KN_set_int_param(lp.solverModel, kn.KN_PARAM_OUTMODE,kn.KN_OUTMODE_FILE)
            elif printKnitroOutput:
                kn.KN_set_int_param(lp.solverModel, kn.KN_PARAM_OUTMODE,kn.KN_OUTMODE_SCREEN)
            else:
                kn.KN_set_int_param(lp.solverModel, kn.KN_PARAM_OUTLEV,kn.KN_OUTLEV_NONE)
            log.debug("add the variables to the problem")
            nvars = len(lp.variables()) 
            vars_up_bounds = []
            vars_low_bounds = []
            vars_types = []
            var_dict = {}
            count=0
            for var in lp.variables():
                var_dict[var.name] = count
                lowBound = var.lowBound
                if lowBound is None:
                    lowBound = -kn.KN_INFINITY
                upBound = var.upBound
                if upBound is None:
                    upBound = kn.KN_INFINITY
                varType = kn.KN_VARTYPE_CONTINUOUS
                if var.cat == constants.LpInteger and self.mip:
                    varType = kn.KN_VARTYPE_INTEGER
                # only add variable once, ow new variable will be created.
                vars_low_bounds.append(lowBound)
                vars_up_bounds.append(upBound)
                vars_types.append(varType)
                count+=1
            kn.KN_add_vars(lp.solverModel, nvars)
            kn.KN_set_var_lobnds(lp.solverModel,range(nvars), vars_low_bounds)
            kn.KN_set_var_upbnds(lp.solverModel, range(nvars),vars_up_bounds)
            kn.KN_set_var_types(lp.solverModel,range(nvars), vars_types)
            if self.optionsDict.get("warmStart", False):
                # Once lp.variables() has been used at least once in the building of the model.
                # we can use the lp._variables with the cache.
                for var in lp._variables:
                    if var.varValue is not None:
                        var.solverVar.start = var.varValue
            log.debug("add the Constraints to the problem")
            ncons = lp.numConstraints()
            count=0
            kn.KN_add_cons(lp.solverModel, ncons)
            for name, constraint in lp.constraints.items():
                # build the expression
                constraint_val=list(constraint.values())
                solvevar=[v for v in constraint.keys()]
                if constraint.sense == constants.LpConstraintLE:
                    kn.KN_set_con_upbnds(lp.solverModel, count,-constraint.constant)

                elif constraint.sense == constants.LpConstraintGE:
                    kn.KN_set_con_lobnds(lp.solverModel, count, -constraint.constant)
                elif constraint.sense == constants.LpConstraintEQ:
                    kn.KN_set_con_eqbnds(lp.solverModel, count, -constraint.constant)
                else:
                    raise PulpSolverError("Detected an invalid constraint type")
                for var in solvevar:
                    # set the coefficient of the variable in the constraint
                    kn.KN_add_con_linear_term(lp.solverModel, count, var_dict[var.name], constraint[var])
                count+=1
            # add objective
            log.debug("add the Objective to the problem")
            obj_indices = []
            obj_coefs = []
            for var in lp.objective.keys():
                obj_indices.append(var_dict[var.name])
                obj_coefs.append(lp.objective[var])
            kn.KN_add_obj_linear_struct(lp.solverModel, obj_indices, obj_coefs)


        def actualSolve(self, lp, callback=None):
            """
            Solve a well formulated lp problem

            creates a Knitro model, variables and constraints and attaches
            them to the lp model which it then solves
            """
            self.buildSolverModel(lp)
            # set the initial solution
            log.debug("Solve the Model using Knitro")
            self.callSolver(lp, callback=callback)
            # get the solution information
            solutionStatus = self.findSolutionValues(lp)
            for var in lp._variables:
                var.modified = False
            for constraint in lp.constraints.values():
                constraint.modified = False
            return solutionStatus

        def actualResolve(self, lp, callback=None):
            """
            Solve a well formulated lp problem

            uses the old solver and modifies the rhs of the modified constraints
            """
            log.debug("Resolve the Model using Knitro")
            cont = 0
            for constraint in lp.constraints.values():
                if constraint.modified:
                    if constraint.sense == constants.LpConstraintLE:
                        kn.KN_set_con_upbnds(lp.solverModel, cont, -constraint.constant)
                    elif constraint.sense == constants.LpConstraintGE:
                        kn.KN_set_con_lobnds(lp.solverModel, cont, -constraint.constant)
                    elif constraint.sense == constants.LpConstraintEQ:
                        kn.KN_set_con_eqbnds(lp.solverModel, cont, -constraint.constant)
            self.callSolver(lp, callback=callback)
            # get the solution information
            solutionStatus = self.findSolutionValues(lp)
            for var in lp._variables:
                var.modified = False
            for constraint in lp.constraints.values():
                constraint.modified = False
            return solutionStatus