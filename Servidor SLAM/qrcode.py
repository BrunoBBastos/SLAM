import math
import cv2

class qrcode:
    # default
    def __init__(self):
        self.valor = -1
        self.pospxl = [-1, -1]
        self.angulo = -1
        self.distancia = -1
        self.qr_dict = {}

    def generate_qr_id(self):
        # O ID é baseado no número de QRs já detectados
        qr_id = len(self.qr_dict) + 1
        return qr_id

    # detect and decod
    def detectAnddecod(self, frame, ret):

        def obj_dist(h_im, res):
            fov = 65  # Campo de visão da camera (degrees)
            w_real = 5  # Largura real do objeto (cm).
            h_real = 5  # Altura real do objeto (cm).
            D_real = math.sqrt(w_real**2 + h_real**2)
            dens = h_im/h_real  # Densidade de pixel por cm

            # Calculo da distancia focal da camera
            d = (res[0]**2 + res[1]**2)**(1/2)
            f = d/2 * (1/(math.tan(math.radians(fov/2))))

            # Calculo distancia do objeto (pixels)
            D_im = math.sqrt(h_im**2 * (1 + (w_real/h_real)**2))
            Dist = (f * D_real) / D_im
            return Dist, dens

        def obj_ang(res, C, dist, dens):
            C_img = (res[0]/2, res[1]/2)  # obtendo o centro da imagem
            d = C_img[0] - C[0]
            theta = math.atan2(d, (dist*dens))
            return theta

        def img_processiment(frame):
            resol_i = frame.shape
            resol = (resol_i[1], resol_i[0])
            img_bl = cv2.blur(frame, (3, 3))
            img_lp = cv2.Laplacian(img_bl, cv2.CV_8U)
            frame = cv2.subtract(frame, img_lp)
            return frame, resol

        if (ret):
            qcd = cv2.QRCodeDetector()
            frame, res = img_processiment(frame)
            read_qr, decoded_info, pontos, _ = qcd.detectAndDecodeMulti(frame)

            if read_qr:  # Se a leitura for possivel
                for valor, pnt in zip(decoded_info, pontos):
                    if valor:
                        if valor in self.qr_dict:
                            # Obtém o ID do QR code existente
                            qr_id = self.qr_dict[valor]
                            # Defindo o ponto central do qr code
                            C = ((sum(pnt[:, 0])/4).astype(int),
                                 (sum(pnt[:, 1])/4).astype(int))

                            self.pospxl = C
                            # Selecionando cor verde (HSV)
                            # para a borda e ponto central
                            color = (0, 255, 0)
                            # inserindo ponto verde no centro do qr_code
                            frame = cv2.circle(frame, (C), 0, color, 5)

                            h_max = max(math.dist(pnt[0, :], pnt[3, :]), math.dist(
                                pnt[1, :], pnt[2, :]))

                            D, dens_pixel = obj_dist(h_max, res)
                            self.distancia = D

                            theta = obj_ang(res, C, D, dens_pixel)
                            self.angulo = theta

                            frame = cv2.putText(
                                frame, str(round(D, 2))+" cm", (C[0]-15, max(pnt[:, 1]).astype(int)+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            frame = cv2.putText(
                                frame, str(round(theta, 2))+"o", (C[0]-15, max(pnt[:, 1]).astype(int)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                            # print("QR code já existente. ID:", qr_id)
                        else:

                            qr_id = self.generate_qr_id()  # Gera um novo ID para o QR code
                            # Insere o valor e o ID no dicionário
                            self.qr_dict[valor] = qr_id

                            # Retornando o valor lido
                            self.valor = valor
                            # Defindo o ponto central do qr code
                            C = ((sum(pnt[:, 0])/4).astype(int),
                                 (sum(pnt[:, 1])/4).astype(int))

                            self.pospxl = C
                            # Selecionando cor verde (HSV)
                            # para a borda e ponto central
                            color = (0, 255, 0)
                            # inserindo ponto verde no centro do qr_code
                            frame = cv2.circle(frame, (C), 0, color, 5)

                            h_max = max(math.dist(pnt[0, :], pnt[3, :]), math.dist(
                                pnt[1, :], pnt[2, :]))

                            D, dens_pixel = obj_dist(h_max, res)
                            self.distancia = D

                            theta = obj_ang(res, C, D, dens_pixel)
                            self.angulo = theta

                            frame = cv2.putText(
                                frame, str(round(D, 2))+" cm", (C[0]-15, max(pnt[:, 1]).astype(int)+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            frame = cv2.putText(
                                frame, str(round(theta, 2))+"o", (C[0]-15, max(pnt[:, 1]).astype(int)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    else:
                        # Selecionando cor Vermelha (HSV) para a borda
                        color = (0, 0, 255)
                        # Retorna 0 ao indentificar mas não decodificar
                        return (0)
                    # Desenhando a borda no Qr Code
                    frame = cv2.polylines(
                        frame, [pnt.astype(int)], True, color, 3)
                    # Desenhando o Centro da imagem
                    frame = cv2.circle(
                        frame, (int(res[0]/2), int(res[1]/2)), 0, (0, 255, 255), 5)

            else:
                # Se leitura não for possivel, retorna '-1'
                return (-1)

            return ([self.distancia, self.angulo, qr_id])
        else:
            # Se leitura do arquivo não for possivel, retorna '-1'
            return (-1)