"""动态集合增量观察的完整性检查；拒绝丢失pop/非法增删/不一致末态。"""
import copy
import unittest
from GmshFrontGenerationComparison import validate_generation_sets


class GenerationSetObservationTest(unittest.TestCase):
    def document(self):
        return {'initial_all':[2,0],'initial_active':[2,0],
                'events':[{'selection':[2],'all_delta':[[],[1]],'active_delta':[[2],[1]]},
                          {'selection':[1],'all_delta':[[],[]],'active_delta':[[1],[]]},
                          {'selection':[0],'all_delta':[[],[]],'active_delta':[[0],[]]}],
                'final':{'all':[1,0,2],'active':[]},'stats':[3]}

    def test_new_member_can_be_popped_after_feedback(self):
        validate_generation_sets(self.document())

    def test_unavailable_pop_is_rejected(self):
        document = self.document(); document['events'][1]['selection'] = [9]
        with self.assertRaisesRegex(ValueError,'popped triangle absent'): validate_generation_sets(document)

    def test_invalid_addition_and_removal_are_rejected(self):
        for key,delta in [('all_delta',[[],[2]]),('active_delta',[[9],[]]),('all_delta',[[],[1,1]])]:
            with self.subTest(key=key,delta=delta):
                document = copy.deepcopy(self.document()); document['events'][0][key] = delta
                with self.assertRaises(ValueError): validate_generation_sets(document)

    def test_final_membership_and_event_count_must_match(self):
        for field,value in [('final',{'all':[2,0],'active':[]}),('stats',[2])]:
            with self.subTest(field=field):
                document = self.document(); document[field] = value
                with self.assertRaises(ValueError): validate_generation_sets(document)


if __name__ == '__main__': unittest.main()
