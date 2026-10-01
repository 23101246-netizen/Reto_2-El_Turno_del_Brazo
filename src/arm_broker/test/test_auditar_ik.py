""" Pruebas de herramientas/auditar_ik.py (ítem 4) — sin robot """

"""Se prueban la matemática del error, la conversión grados/radianes, el registro de evidencia y
el flujo completo con un brazo simulado que se comporta como un MyCobot

    cd src/arm_broker && python3 -m unittest discover -s test -v
"""
import csv
import math
import os
import sys
import tempfile
import unittest

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
sys.path.insert(0, os.path.join(RAIZ, 'src', 'arm_broker'))
sys.path.insert(0, os.path.join(RAIZ, 'herramientas'))

import auditar_ik as A                                    # noqa: E402
from arm_broker import fk                                 # noqa: E402


class BrazoFalso:
    """Se comporta como un MyCobot: send_coords mueve a una pose q, y get_angles/get_coords la leen"""

    def __init__(self, q_deg, coords=None):
        self.q_deg = list(q_deg) if isinstance(q_deg, (list, tuple)) else q_deg
        self.coords = coords
        self.enviados = []

    def send_coords(self, coords, velocidad, modo):
        self.enviados.append((list(coords), velocidad, modo))

    def get_angles(self):
        return self.q_deg

    def get_coords(self):
        if self.coords is not None:
            return self.coords
        return list(fk.fk([math.radians(v) for v in self.q_deg])) + [0.0, 0.0, 0.0]


Q_DEG = [10.0, -25.0, 30.0, 5.0, 20.0, 0.0]


class TestMatematica(unittest.TestCase):
    def test_error_345(self):
        self.assertEqual(A.error_cartesiano((0, 0, 0), (3, 4, 0)), 5.0)

    def test_error_ejemplo_del_enunciado(self):
        e = A.error_cartesiano((200.0, 50.0, 180.0), (198.0, 53.0, 184.0))
        self.assertAlmostEqual(e, math.sqrt(29))

    def test_error_cero_y_simetria(self):
        p, q = (10.0, -20.0, 30.0), (13.0, -16.0, 30.0)
        self.assertEqual(A.error_cartesiano(p, p), 0.0)
        self.assertAlmostEqual(A.error_cartesiano(p, q), A.error_cartesiano(q, p))

    def test_error_exige_tres_coordenadas(self):
        with self.assertRaises(ValueError):
            A.error_cartesiano((1, 2), (1, 2, 3))

    def test_errores_por_eje_son_obtenido_menos_objetivo(self):
        self.assertEqual(A.errores_por_eje((200, 50, 180), (198, 53, 184)), (-2, 3, 4))

    def test_grados_a_radianes(self):
        q = A.grados_a_radianes([0, 90, 180, -90, 45, 360])
        esperado = [0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4, 2 * math.pi]
        for a, b in zip(q, esperado):
            self.assertAlmostEqual(a, b)

    def test_ida_y_vuelta(self):
        for a, b in zip(A.radianes_a_grados(A.grados_a_radianes(Q_DEG)), Q_DEG):
            self.assertAlmostEqual(a, b)

    def test_fk_con_q_en_grados_convertido_coincide_con_fk_en_radianes(self):
        q_rad = [math.radians(v) for v in Q_DEG]
        self.assertEqual(fk.fk(q_rad), fk.fk(A.grados_a_radianes(Q_DEG)))


class TestAuditoria(unittest.TestCase):
    def test_objetivo_igual_a_la_fk_da_error_cero(self):
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        r = A.auditar(Q_DEG, list(xyz) + [0, 0, 0], xyz, (0, 0, 0), 30)
        self.assertAlmostEqual(r['error_mm'], 0.0)
        self.assertAlmostEqual(r['dif_fk_robot_mm'], 0.0)

    def test_error_conocido(self):
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        objetivo = (xyz[0] + 3.0, xyz[1] - 4.0, xyz[2])
        r = A.auditar(Q_DEG, list(xyz) + [0, 0, 0], objetivo)
        self.assertAlmostEqual(r['error_mm'], 5.0)
        self.assertAlmostEqual(r['error_ejes'][0], -3.0)
        self.assertAlmostEqual(r['error_ejes'][1], 4.0)

    def test_la_fk_audita_q_real_no_get_coords(self):
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        desfasado = [xyz[0] + 20.0, xyz[1], xyz[2], 0, 0, 0]       # get_coords() con otro marco
        r = A.auditar(Q_DEG, desfasado, xyz)
        self.assertAlmostEqual(r['error_mm'], 0.0)                 # contra el objetivo: FK(q_real)
        self.assertAlmostEqual(r['dif_fk_robot_mm'], 20.0)         # y el desfase queda visible

    def test_fila_csv_tiene_una_columna_por_encabezado(self):
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        r = A.auditar(Q_DEG, list(xyz) + [0, 0, 0], xyz, (1, 2, 3), 30)
        self.assertEqual(len(A.registro_a_fila(r)), len(A.COLUMNAS))
        self.assertEqual(A.COLUMNAS[:3], ['x_obj', 'y_obj', 'z_obj'])
        self.assertEqual(A.COLUMNAS[15], 'error_mm')

    def test_guardar_csv_crea_encabezado_y_agrega_filas(self):
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        r = A.auditar(Q_DEG, list(xyz) + [0, 0, 0], xyz)
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, 'item4', 'auditoria_ik.csv')
            A.guardar_csv(r, ruta)
            A.guardar_csv(r, ruta)
            with open(ruta, newline='', encoding='utf-8') as f:
                filas = list(csv.DictReader(f))
        self.assertEqual(len(filas), 2)
        self.assertEqual(list(filas[0]), A.COLUMNAS)

    def test_tabla_vacia_dice_pendiente_y_con_datos_no(self):
        vacia = A.tabla_evidencia()
        self.assertEqual(vacia.count('pendiente'), 11)
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        r = A.auditar(Q_DEG, list(xyz) + [0, 0, 0], xyz, (0, 0, 0), 30)
        self.assertNotIn('pendiente', A.tabla_evidencia(r))


class TestFlujoConBrazoSimulado(unittest.TestCase):
    def test_envia_send_coords_y_audita_el_q_leido(self):
        brazo = BrazoFalso(Q_DEG)
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        objetivo = (xyz[0] + 2.0, xyz[1] + 3.0, xyz[2] + 6.0)
        r = A.ejecutar(brazo, objetivo, (0.0, 90.0, 0.0), velocidad=25, modo=0, espera_s=0.0)
        self.assertEqual(brazo.enviados, [(list(objetivo) + [0.0, 90.0, 0.0], 25, 0)])
        self.assertAlmostEqual(r['error_mm'], 7.0)                  # sqrt(4 + 9 + 36)

    def test_solo_leer_no_manda_send_coords(self):
        brazo = BrazoFalso(Q_DEG)
        A.ejecutar(brazo, fk.fk(A.grados_a_radianes(Q_DEG)), mover=False)
        self.assertEqual(brazo.enviados, [])

    def test_mover_sin_orientacion_es_un_error(self):
        with self.assertRaises(ValueError):
            A.ejecutar(BrazoFalso(Q_DEG), (200, 50, 180), None, espera_s=0.0)

    def test_brazo_que_no_responde(self):
        for q in ([], -1, None, [1, 2, 3]):
            with self.assertRaises(RuntimeError):
                A.leer_estado(BrazoFalso(q, coords=[0, 0, 0, 0, 0, 0]))
        with self.assertRaises(RuntimeError):
            A.leer_estado(BrazoFalso(Q_DEG, coords=-1))

    def test_main_no_mueve_sin_objetivo_ni_confirmacion(self):
        self.assertEqual(A.main([]), 2)
        self.assertEqual(A.main(['--x', '200', '--y', '50', '--z', '180',
                                 '--rx', '0', '--ry', '0', '--rz', '0']), 2)

    def test_main_plantilla_no_necesita_robot(self):
        self.assertEqual(A.main(['--plantilla']), 0)

    def test_advertencia_de_objetivo_fuera_de_alcance(self):
        self.assertTrue(A.advertencias_objetivo((900.0, 0.0, 0.0)))
        self.assertEqual(A.advertencias_objetivo((200.0, 50.0, 180.0)), [])
        with self.assertRaises(ValueError):
            A.advertencias_objetivo((float('nan'), 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
