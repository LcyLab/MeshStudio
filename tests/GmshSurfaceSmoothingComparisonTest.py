"""平滑观察证据的完整性检查：错误发布、漏试探和查询漏记必须显式失败。"""
import copy
import unittest
from GmshSurfaceSmoothingComparison import validate_result, generation_smoothing_input


def rejected_fan():
    events = [{"iteration":0,"vertex":i,"skip":3,"before":[0.,0.],"position":[0.,0.,0.],"neighbors":[],"trials":[]} for i in (1,2,3)]
    event = {"iteration":0,"vertex":7,"skip":0,"before":[.1,.2],"position":[.5,.5,3.],
             "neighbors":[[i,0.,0.,0.,0.,1.,0.,1.,1.] for i in (1,2,3)],"trials":[]}
    for i in range(5):
        event["trials"].append([1./(1.4**i),[.5,.5],False,False,[0.,0.,0.],0.,0.,1.,1.,False,False,[0.,0.,0.],[.1,.2],1])
    events.append(event)
    return {"events":events,"triangles":[[1,2,7],[2,3,7],[3,1,7]],"visits":4,"eligible":1,"writes":0,"queries":14}


class SmoothingEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.result = rejected_fan()

    def test_valid_observations_and_zero_iterations(self):
        validate_result(self.result,1)
        validate_result({"events":[],"triangles":[],"visits":0,"eligible":0,"writes":0,"queries":0},0)

    def test_success_does_not_allow_early_exit(self):
        self.result["events"][-1]["trials"].pop()
        with self.assertRaisesRegex(ValueError,"five trials"):
            validate_result(self.result,1)

    def test_failed_point_cannot_publish(self):
        self.result["events"][-1]["trials"][0][9] = True
        with self.assertRaisesRegex(ValueError,"published"):
            validate_result(self.result,1)

    def test_unpublished_trial_cannot_change_actual(self):
        self.result["events"][-1]["trials"][1][12] = [.5,.5]
        with self.assertRaisesRegex(ValueError,"actual position"):
            validate_result(self.result,1)

    def test_acceptance_queries_cannot_be_omitted(self):
        self.result["events"][-1]["trials"][0][3] = True
        with self.assertRaisesRegex(ValueError,"raw reparameterization"):
            validate_result(self.result,1)

    def test_global_query_count_cannot_hide_a_missing_call(self):
        self.result["queries"] -= 1
        with self.assertRaisesRegex(ValueError,"summary"):
            validate_result(self.result,1)

    def test_native_identity_order_is_required(self):
        result = copy.deepcopy(self.result)
        result["events"][-1]["neighbors"].reverse()
        with self.assertRaisesRegex(ValueError,"neighbors"):
            validate_result(result,1)
        self.result["events"][0],self.result["events"][1] = self.result["events"][1],self.result["events"][0]
        with self.assertRaisesRegex(ValueError,"vertex visits"):
            validate_result(self.result,1)

    def test_prefix_preserves_original_stored_parameter_instead_of_seam_slot(self):
        mesh = {'vertices':[[7,7,0,1,3,1.,2.,3.,.25,0.,-1.],[9,7,1,2,4,1.,2.,3.,6.28,.4,-1.]]}
        generation = {'final':{'slots':[[7,6.28,.3]],'live_indices':[[7,0]]},'vertices':[[7,1.,2.,3.,1]]}
        nodes,triangles = generation_smoothing_input(4,generation,{'triangles':[[7,9,7]]},mesh)
        self.assertEqual(nodes[0][6:10],[.25,0.,1,0])
        self.assertEqual(nodes[1][6:10],[6.28,.4,1,1])
        self.assertEqual(triangles,[[7,9,7]])

    def test_prefix_new_vertex_uses_successful_new_face_slot(self):
        generation = {'final':{'slots':[[11,.3,.4]],'live_indices':[[11,0]]},'vertices':[[11,1.,2.,3.,2]]}
        nodes,_ = generation_smoothing_input(4,generation,{'triangles':[]},{'vertices':[]})
        self.assertEqual(nodes,[[11,2,4,1.,2.,3.,.3,.4,1,1,0]])

    def test_prefix_missing_new_vertex_parameters_is_not_fabricated(self):
        generation = {'final':{'slots':[],'live_indices':[]},'vertices':[[11,1.,2.,3.,2]]}
        with self.assertRaisesRegex(ValueError,'stored UV'):
            generation_smoothing_input(4,generation,{'triangles':[]},{'vertices':[]})


if __name__ == "__main__":
    unittest.main()
