add_artery_feature(mcm
    BaseMcService.cc
    McGeometry.cc
    McLog.cc
    McService.cc
    McTrajectoryPlanner.cc
)

add_opp_message(mcm-message MESSAGE Mcm.msg)
set_target_properties(mcm-message PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mcm PUBLIC mcm-message)
