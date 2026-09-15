import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SedsnetRoutingContract(unittest.TestCase):
    def test_only_local_loadcell_stream_has_a_can_route_override(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "Core" / "Src").glob("*.c")
        )
        self.assertNotIn("seds_router_set_route", sources)
        self.assertEqual(sources.count("seds_router_set_typed_route("), 1)
        self.assertIn("seds_router_set_typed_route(r, -1, (uint32_t)SEDS_DT_KG1000,\n"
                      "                                       g_can_side_id, true)", sources)


if __name__ == "__main__":
    unittest.main()
