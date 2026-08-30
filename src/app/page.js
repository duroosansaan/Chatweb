import { connectDB } from '../lib/mongodb';
import Subscriber from '../models/Subscriber';

export default async function Dashboard() {
  await connectDB();
  const subscribers = await Subscriber.find({}).sort({ lastInteraction: -1 }).lean();

  return (
    <div className="min-h-screen bg-gray-50 p-8" dir="rtl">
      <div className="max-w-4xl mx-auto">
        <h1 className="text-3xl font-bold text-gray-800 mb-6 text-right">لوحة تحكم العملاء (Leads)</h1>
        
        <div className="bg-white rounded-lg shadow overflow-hidden">
          <table className="w-full text-right">
            <thead className="bg-gray-100 border-b">
              <tr>
                <th className="p-4 font-semibold text-gray-600">رقم الهاتف</th>
                <th className="p-4 font-semibold text-gray-600">المرحلة</th>
                <th className="p-4 font-semibold text-gray-600">رقم الحساب (ID)</th>
              </tr>
            </thead>
            <tbody>
              {subscribers.map((sub, index) => (
                <tr key={index} className="border-b hover:bg-gray-50">
                  <td className="p-4 font-medium text-blue-600" dir="ltr">{sub.phone}</td>
                  <td className="p-4">
                    <span className={`px-3 py-1 rounded-full text-sm ${sub.step === 'FINISHED' ? 'bg-green-100 text-green-700' : 'bg-yellow-100 text-yellow-700'}`}>
                      {sub.step === 'FINISHED' ? 'مكتمل' : 'قيد المحادثة'}
                    </span>
                  </td>
                  <td className="p-4 text-gray-500 text-sm">{sub.platformId}</td>
                </tr>
              ))}
            </tbody>
          </table>
          
          {subscribers.length === 0 && (
            <div className="p-8 text-center text-gray-500">لا يوجد عملاء حتى الآن.</div>
          )}
        </div>
      </div>
    </div>
  );
}
