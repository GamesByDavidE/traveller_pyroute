"""
Created on Sep 12, 2023

@author: CyberiaResurrection
"""
import os

from pypdflite import PDFCursor, PDFLite
from pypdflite.pdfobjects.pdfellipse import PDFEllipse
from pypdflite.pdfobjects.pdfline import PDFLine
from pypdflite.pdfobjects.pdftext import PDFText

from PyRoute.Outputs.Map import Map
from PyRoute.StatCalculation import StatCalculation


class PDFSectorMap(Map):
    def __init__(self, galaxy, routes):
        super(PDFSectorMap, self).__init__(galaxy, routes)
        self.lineStart = PDFCursor(0, 0)
        self.lineEnd = PDFCursor(0, 0)

    def document(self, sector):
        """
        Generated by the type of document
        """
        path = os.path.join(self.galaxy.output_path, sector.sector_name() + " Sector.pdf")
        self.writer = PDFLite(path)

        title = "Sector %s" % sector
        subject = "Trade route map generated by PyRoute for Traveller"
        author = None
        keywords = None
        creator = "PyPDFLite"
        self.writer.set_information(title, subject, author, keywords, creator)
        document = self.writer.get_document()
        document.set_margins(4)
        return document

    def close(self):
        self.writer.close()

    def cursor(self, x=0, y=0):
        return PDFCursor(x, y)

    def sector_name(self, doc, name):
        """
        Write name at the top of the document
        """
        cursor = PDFCursor(5, -5, True)
        def_font = doc.get_font()
        doc.set_font('times', size=30)
        width = doc.get_font()._string_width(name)
        cursor.x = 306 - (width / 2)
        doc.add_text(name, cursor)
        doc.set_font(font=def_font)

    def coreward_sector(self, pdf, name):
        cursor = PDFCursor(5, self.y_start - 15, True)
        def_font = pdf.get_font()
        pdf.set_font('times', size=10)
        width = pdf.get_font()._string_width(name) / 2
        cursor.x = 306 - width
        pdf.add_text(name, cursor)
        pdf.set_font(font=def_font)

    def rimward_sector(self, pdf, name):
        cursor = PDFCursor(306, 767, True)
        def_font = pdf.get_font()
        pdf.set_font('times', size=10)
        cursor.x_plus(-pdf.get_font()._string_width(name) / 2)
        pdf.add_text(name, cursor)
        pdf.set_font(font=def_font)

    def spinward_sector(self, pdf, name):
        cursor = PDFCursor(self.x_start - 5, 396, True)
        def_font = pdf.get_font()
        pdf.set_font('times', size=10)
        cursor.y_plus(pdf.get_font()._string_width(name) / 2)
        text = PDFText(pdf.session, pdf.page, None, cursor=cursor)
        text.text_rotate(90)
        text._text(name)
        pdf.set_font(font=def_font)

    def trailing_sector(self, pdf, name):
        cursor = PDFCursor(598, 396 - self.y_start, True)
        def_font = pdf.get_font()
        pdf.set_font('times', size=10)
        cursor.y_plus(-(pdf.get_font()._string_width(name) / 2))
        text = PDFText(pdf.session, pdf.page, None, cursor=cursor)
        text.text_rotate(-90)
        text._text(name)
        pdf.set_font(font=def_font)

    def add_line(self, pdf, start, end, colorname):
        """
        Add a line to the document, from start to end, in color
        """
        color = pdf.get_color()
        color.set_color_by_name(colorname)
        pdf.set_draw_color(color)
        pdf.add_line(cursor1=start, cursor2=end)

    def add_circle(self, pdf, center, radius, colorname):
        color = pdf.get_color()
        color.set_color_by_name(colorname)
        radius = PDFCursor(radius, radius)
        circle = PDFEllipse(pdf.session, pdf.page, center, radius, color, size=2)
        circle._draw()

    def get_line(self, doc, start, end, colorname, width):
        """
        Get a line draw method processor
        """
        color = doc.get_color()
        color.set_color_by_name(colorname)
        return PDFLine(doc.session, doc.page, start, end, stroke='solid', color=color, size=width)

    def place_system(self, pdf, star):
        def_font = pdf.get_font()
        pdf.set_font('times', size=4)

        col = (self.xm * 3 * (star.col))
        if (star.col & 1):
            row = (self.y_start - self.ym * 2) + (star.row * self.ym * 2)
        else:
            row = (self.y_start - self.ym) + (star.row * self.ym * 2)

        point = PDFCursor(col, row)
        self.zone(pdf, star, point.copy())

        width = self.string_width(pdf.get_font(), star.uwp)
        point.y_plus(7)
        point.x_plus(self.ym - (width / 2))
        pdf.add_text(star.uwp.encode('ascii', 'replace'), point)

        if len(star.name) > 0:
            for chars in range(len(star.name), 0, -1):
                width = self.string_width(pdf.get_font(), star.name[:chars])
                if width <= self.xm * 3.5:
                    break
            point.y_plus(3.5)
            point.x = col
            point.x_plus(self.ym - (width / 2))
            pdf.add_text(star.name[:chars].encode('ascii', 'replace'), point)

        added = star.alg
        if 'Cp' in star.tradeCode:
            added += '+'
        elif 'Cx' in star.tradeCode or 'Cs' in star.tradeCode:
            added += '*'
        else:
            added += ' '

        added += '{:d}'.format(star.ggCount)
        point.y_plus(3.5)
        point.x = col
        width = pdf.get_font()._string_width(added)
        point.x_plus(self.ym - (width / 2))
        pdf.add_text(added, point)

        added = ''
        tradeIn = StatCalculation.trade_to_btn(star.tradeIn)
        tradeThrough = StatCalculation.trade_to_btn(star.tradeIn + star.tradeOver)

        if self.routes == 'trade':
            added += "{:X}{:X}{:X}{:d}".format(star.wtn, tradeIn, tradeThrough, star.starportSize)
        elif self.routes == 'comm':
            added += "{}{} {}".format(star.baseCode, star.ggCount, star.importance)
        elif self.routes == 'xroute':
            added += " {}".format(star.importance)
        width = pdf.get_font()._string_width(added)
        point.y_plus(3.5)
        point.x = col
        point.x_plus(self.ym - (width / 2))
        pdf.add_text(added, point)

        pdf.set_font(def_font)
